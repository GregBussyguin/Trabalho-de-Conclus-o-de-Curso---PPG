#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <math.h>

static const char *NOME_BLE = "PPG-ESP32";
#define UUID_SERVICO "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
#define UUID_RX      "6e400002-b5a3-f393-e0a9-e50e24dcca9e"

static const uint16_t MTU_ALVO = 247;

bool g_bleDeviceConnected = false;
float g_bpmBLE = 0.0f;

const uint8_t FONT_SIZE_HEADER      = 1;
const uint8_t FONT_SIZE_LABEL       = 1;
const uint8_t FONT_SIZE_VALUE       = 2;
const uint8_t FONT_SIZE_UNIT        = 1;
const uint16_t TEXT_COLOR_DEFAULT   = SSD1306_WHITE;

const char* STR_BOOT_TITLE          = "PPG METER";
const char* STR_BOOT_SUBTITLE       = "UTFPR";
const char* STR_TITLE_MONITOR       = "== MONITOR DE PPG ==";
const char* STR_LABEL_PPG_LOC       = "PPG Local:";
const char* STR_LABEL_PPG_BLE       = "PPG Remoto (BLE):";
const char* STR_UNIT_BPM            = "BPM";
const char* STR_TITLE_STATS1        = "== COMPARACAO PPG ==";
const char* STR_TITLE_STATS2        = "== MSE: PPG vs BLE ==";

#define PIN_PPG_ADC          20   

#define PIN_OFFSET_PWM       4    
#define PIN_LED_CTRL         47   

#define PIN_ENCODER_CLK      10   
#define PIN_ENCODER_DT       11   
#define PIN_ENCODER_SW       12   

#define PIN_OLED_SDA         8    
#define PIN_OLED_SCL         9    

#define PWM_FREQ_HZ          5000
#define PWM_RESOLUTION_BITS  12
#define PWM_INITIAL_DUTY     1923.0f 

#define SCREEN_WIDTH         128
#define SCREEN_HEIGHT        64
#define OLED_RESET           -1
#define OLED_I2C_ADDRESS     0x3C

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
bool g_oledOk = false;

#define SAMPLING_FREQ_HZ     250
#define SAMPLING_PERIOD_US   (1000000 / SAMPLING_FREQ_HZ)
#define ADC_TARGET_OFFSET    2048.0f 
#define PLOT_DIVIDER         5 

#define INVERT_PI_POLARITY   false
#define PWM_MIN_DUTY         200.0f
#define PWM_MAX_DUTY         3200.0f

struct PIOffsetController {
  float Kp = 0.0004f;
  float Ki = 0.00002f;
  float integrator = PWM_INITIAL_DUTY;
  bool enabled = true;

  float update(float target, float measured) {
    if (!enabled) return PWM_INITIAL_DUTY;
    float error = INVERT_PI_POLARITY ? (measured - target) : (target - measured);
    integrator += error * Ki;
    integrator = constrain(integrator, PWM_MIN_DUTY, PWM_MAX_DUTY);
    float output = (error * Kp) + integrator;
    return constrain(output, PWM_MIN_DUTY, PWM_MAX_DUTY);
  }
} g_piOffset;

class NotchFilter60Hz {
private:
  float x1 = 0.0f, x2 = 0.0f;
  float y1 = 0.0f, y2 = 0.0f;
public:
  void reset() {
    x1 = x2 = 0.0f;
    y1 = y2 = 0.0f;
  }
  float process(float x) {
    float y = 0.905335f * (x - 0.125581f * x1 + x2) + 0.113023f * y1 - 0.810000f * y2;
    x2 = x1; x1 = x;
    y2 = y1; y1 = y;
    return y;
  }
};

class SmoothPPGFilter {
private:
  float buf[5] = {0.0f};
  uint8_t idx = 0;
  float dcTracker = 2048.0f;
  NotchFilter60Hz notch;
public:
  void reset() {
    for (uint8_t i = 0; i < 5; i++) buf[i] = 0.0f;
    idx = 0;
    dcTracker = 2048.0f;
    notch.reset();
  }
  float process(float rawInput) {
    dcTracker = 0.998f * dcTracker + 0.002f * rawInput;
    float acSignal = rawInput - dcTracker;
    float notched = notch.process(acSignal);
    buf[idx] = notched;
    idx = (idx + 1) % 5;
    float sum = 0.0f;
    for (uint8_t i = 0; i < 5; i++) sum += buf[i];
    return sum / 5.0f;
  }
};

class PeakDetector {
private:
  float threshold;
  uint32_t lastPeakMs;
  float alpha;
  uint32_t rrIntervals[10];
  uint8_t rrIdx;
  float maxValInPeak;
  bool trackingPeak;
  float lastValidBPM;
  uint32_t lastRefrPeriodMs;

public:
  PeakDetector(float initThresh = 30.0f, float initAlpha = 0.15f) 
    : threshold(initThresh), lastPeakMs(0), alpha(initAlpha), rrIdx(0),
      maxValInPeak(0.0f), trackingPeak(false), lastValidBPM(0.0f), lastRefrPeriodMs(320) {
    for (int i = 0; i < 10; i++) rrIntervals[i] = 0;
  }

  void reset() {
    lastPeakMs = 0;
    trackingPeak = false;
    lastValidBPM = 0.0f;
    threshold = 30.0f;
    rrIdx = 0;
    lastRefrPeriodMs = 320;
    for (int i = 0; i < 10; i++) rrIntervals[i] = 0;
  }

  bool processSample(float value, uint32_t timestampMs, float &outBPM) {
    bool peakDetected = false;

    // Timeout se ficar mais de 3.5 segundos sem nenhum batimento (dedo retirado)
    if (lastPeakMs > 0 && (timestampMs - lastPeakMs > 3500)) {
      lastValidBPM = 0.0f;
    }

    // Período refratário adaptativo baseado no RR médio anterior (evita contagem dupla por onda dicrótica)
    uint32_t refr = lastRefrPeriodMs;
    if (refr < 300) refr = 300;
    if (refr > 700) refr = 700;

    // Detecta início do pico se cruzar o limiar e respeitar o período refratário
    if (value > threshold && !trackingPeak && (lastPeakMs == 0 || (timestampMs - lastPeakMs > refr))) {
      trackingPeak = true;
      maxValInPeak = value;
    }

    if (trackingPeak) {
      if (value > maxValInPeak) {
        maxValInPeak = value;
      } 
      else if (value < (maxValInPeak * 0.80f)) {
        trackingPeak = false;
        
        if (lastPeakMs > 0) {
          uint32_t interval = timestampMs - lastPeakMs;

          if (interval >= 280 && interval <= 2200) {
            uint32_t sumRR = 0;
            int countRR = 0;
            for (int i = 0; i < 10; i++) {
              if (rrIntervals[i] > 0) {
                sumRR += rrIntervals[i];
                countRR++;
              }
            }

            uint32_t currentAvgRR = (countRR > 0) ? (sumRR / countRR) : interval;
            
            // Rejeição suave de outliers
            if (abs((int32_t)interval - (int32_t)currentAvgRR) < (currentAvgRR * 0.6f)) {
              rrIntervals[rrIdx] = interval;
            } else {
              rrIntervals[rrIdx] = (currentAvgRR + interval) / 2;
            }
            rrIdx = (rrIdx + 1) % 10;

            sumRR = 0;
            countRR = 0;
            for (int i = 0; i < 10; i++) {
              if (rrIntervals[i] > 0) {
                sumRR += rrIntervals[i];
                countRR++;
              }
            }
            uint32_t newAvgRR = (countRR > 0) ? (sumRR / countRR) : interval;

            float rawBPM = 60000.0f / (float)newAvgRR;
            if (lastValidBPM <= 0.0f) {
              lastValidBPM = rawBPM;
            } else {
              lastValidBPM = (lastValidBPM * 0.80f) + (rawBPM * 0.20f);
            }

            // Atualiza período refratário para 55% do intervalo cardíaco
            lastRefrPeriodMs = (uint32_t)(newAvgRR * 0.55f);
            peakDetected = true;
          }
        } else {
          // Primeiro pico após ligar/reiniciar
          peakDetected = true;
        }
        
        lastPeakMs = timestampMs;
        threshold = threshold * (1.0f - alpha) + maxValInPeak * alpha * 0.5f;
      }
    } else {
      threshold *= 0.998f;
      if (threshold < 6.0f) threshold = 6.0f;
    }

    outBPM = lastValidBPM;
    return peakDetected;
  }
};

SemaphoreHandle_t xDataMutex = NULL;

static const size_t MAX_LINHA = 96;
char g_linhaBuffer[MAX_LINHA];
size_t g_posLinha = 0;

const char *campo(const char *s, int n) {
  static char buf[32];
  int atual = 0;
  const char *ini = s;
  for (const char *p = s;; ++p) {
    if (*p == ',' || *p == '\0') {
      if (atual == n) {
        size_t len = (size_t)(p - ini);
        if (len >= sizeof(buf)) len = sizeof(buf) - 1;
        memcpy(buf, ini, len);
        buf[len] = '\0';
        return buf;
      }
      if (*p == '\0') break;
      atual++;
      ini = p + 1;
    }
  }
  buf[0] = '\0';
  return buf;
}

float campoF(const char *s, int n) {
  const char *c = campo(s, n);
  return c[0] ? atof(c) : NAN;
}

struct BiomedicalStats {
  float diffPPG_BLE = 0.0f;  
  float sePPG_BLE   = 0.0f;  
  float msePPG_BLE  = 0.0f;  
  float meanPPG     = 0.0f;  

  void update(float bpmPPG, float bpmBLE) {
    int validCount = 0;
    float sumBPM = 0.0f;
    if (bpmPPG >= 30.0f && bpmPPG <= 220.0f) { sumBPM += bpmPPG; validCount++; }
    if (bpmBLE >= 30.0f && bpmBLE <= 220.0f) { sumBPM += bpmBLE; validCount++; }
    meanPPG = (validCount > 0) ? (sumBPM / (float)validCount) : 0.0f;

    if (bpmPPG >= 30.0f && bpmBLE >= 30.0f) {
      float err = bpmPPG - bpmBLE;
      diffPPG_BLE = fabsf(err);
      sePPG_BLE = err * err;
      msePPG_BLE = (msePPG_BLE == 0.0f) ? sePPG_BLE : (0.90f * msePPG_BLE + 0.10f * sePPG_BLE);
    } else {
      diffPPG_BLE = 0.0f;
      sePPG_BLE = 0.0f;
    }
  }
} g_stats;

float g_bpmPPG = 0.0f;

void trataLinhaBLE(const char *s) {
  float tempBPM = 0.0f;
  bool atualizar = false;

  switch (s[0]) {
    case 'H': tempBPM = 0.0f; atualizar = true; break;
    case 'B': 
    case 'E': {
      float bpm = campoF(s, 2);
      if (!isnan(bpm) && bpm >= 30.0f && bpm <= 220.0f) {
        tempBPM = bpm;
        atualizar = true;
      }
      break;
    }
    case 'X': tempBPM = 0.0f; atualizar = true; break;
  }

  if (atualizar && xDataMutex != NULL) {
    if (xSemaphoreTake(xDataMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
      g_bpmBLE = tempBPM;
      g_stats.update(g_bpmPPG, g_bpmBLE);
      xSemaphoreGive(xDataMutex);
    }
  }
}

void consomeBytesBLE(const uint8_t *dados, size_t n) {
  for (size_t i = 0; i < n; i++) {
    char c = (char)dados[i];
    if (c == '\n' || c == '\r') {
      if (g_posLinha > 0) {
        g_linhaBuffer[g_posLinha] = '\0';
        trataLinhaBLE(g_linhaBuffer);
        g_posLinha = 0;
      }
    } else if (g_posLinha < MAX_LINHA - 1) {
      g_linhaBuffer[g_posLinha++] = c;
    } else {
      g_posLinha = 0;
    }
  }
}

enum UIMenuState { 
  UI_STATE_MONITOR = 0, 
  UI_STATE_BPM_COMPARE = 1, 
  UI_STATE_MSE = 2 
};
volatile UIMenuState g_currentMenuState = UI_STATE_MONITOR;

struct PPGSample {
  uint16_t rawPPG;
  float filteredPPG;
};

enum EncoderEvent { 
  EVENT_NONE, 
  EVENT_NEXT_SCREEN, 
  EVENT_PREV_SCREEN,
  EVENT_CLICK
};

QueueHandle_t xSampleQueue = NULL;
QueueHandle_t xUIEventQueue = NULL;
TaskHandle_t xSamplingTaskHandle = NULL;

class ServidorCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *s) override {
    g_bleDeviceConnected = true;
    g_posLinha = 0;
  }
  void onDisconnect(BLEServer *s) override {
    g_bleDeviceConnected = false;
    if (xSemaphoreTake(xDataMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
      g_bpmBLE = 0.0f;
      g_stats.update(g_bpmPPG, g_bpmBLE);
      xSemaphoreGive(xDataMutex);
    }
    BLEDevice::startAdvertising();
  }
};

class RxCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *c) override {
    String rxValue = c->getValue();
    if (rxValue.length() > 0) {
      consomeBytesBLE((const uint8_t*)rxValue.c_str(), rxValue.length());
    }
  }
};

volatile uint32_t g_lastEncoderInterruptTime = 0;
volatile uint32_t g_lastButtonInterruptTime = 0;

void IRAM_ATTR onEncoderClkISR() {
  uint32_t now = millis();
  if (now - g_lastEncoderInterruptTime > 30) {
    int clk = digitalRead(PIN_ENCODER_CLK);
    int dt  = digitalRead(PIN_ENCODER_DT);
    EncoderEvent event = (clk != dt) ? EVENT_NEXT_SCREEN : EVENT_PREV_SCREEN;
    
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    if (xUIEventQueue != NULL) xQueueSendFromISR(xUIEventQueue, &event, &xHigherPriorityTaskWoken);
    if (xHigherPriorityTaskWoken == pdTRUE) portYIELD_FROM_ISR();
    g_lastEncoderInterruptTime = now;
  }
}

void IRAM_ATTR onEncoderSwISR() {
  uint32_t now = millis();
  if (now - g_lastButtonInterruptTime > 200) {
    EncoderEvent event = EVENT_CLICK;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    if (xUIEventQueue != NULL) xQueueSendFromISR(xUIEventQueue, &event, &xHigherPriorityTaskWoken);
    if (xHigherPriorityTaskWoken == pdTRUE) portYIELD_FROM_ISR();
    g_lastButtonInterruptTime = now;
  }
}

hw_timer_t *g_samplingTimer = NULL;
void IRAM_ATTR onSamplingTimerISR() {
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  if (xSamplingTaskHandle != NULL) vTaskNotifyGiveFromISR(xSamplingTaskHandle, &xHigherPriorityTaskWoken);
  if (xHigherPriorityTaskWoken == pdTRUE) portYIELD_FROM_ISR();
}

void vTaskPPGSampling(void *pvParameters) {
  SmoothPPGFilter ppgFilter;
  PeakDetector ppgPeakDetector(30.0f, 0.15f);
  
  digitalWrite(PIN_LED_CTRL, HIGH);
  uint8_t plotCounter = 0;
  
  for (;;) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    uint32_t nowMs = millis();
    
    uint16_t rawPPG = analogRead(PIN_PPG_ADC);
    
    float adjustedDuty = g_piOffset.update(ADC_TARGET_OFFSET, (float)rawPPG);
    ledcWrite(PIN_OFFSET_PWM, (uint32_t)adjustedDuty);
    
    float filteredPPG = ppgFilter.process((float)rawPPG);
    
    float currentBpmPPG = 0.0f;
    bool ppgBeat = false;
    if (ppgPeakDetector.processSample(filteredPPG, nowMs, currentBpmPPG)) {
      if (currentBpmPPG >= 30.0f && currentBpmPPG <= 220.0f) {
        g_bpmPPG = currentBpmPPG;
        ppgBeat = true;
      }
    } else {
      if (currentBpmPPG >= 30.0f && currentBpmPPG <= 220.0f) {
        g_bpmPPG = currentBpmPPG;
      } else if (currentBpmPPG <= 0.0f) {
        if (g_bpmPPG != 0.0f) {
          g_bpmPPG = 0.0f;
          if (xDataMutex != NULL && xSemaphoreTake(xDataMutex, 0) == pdTRUE) {
            g_stats.update(g_bpmPPG, g_bpmBLE);
            xSemaphoreGive(xDataMutex);
          }
        }
      }
    }
    
    if (ppgBeat) {
      if (xDataMutex != NULL && xSemaphoreTake(xDataMutex, 0) == pdTRUE) {
        g_stats.update(g_bpmPPG, g_bpmBLE);
        xSemaphoreGive(xDataMutex);
      }
    }
    
    plotCounter++;
    if (plotCounter >= PLOT_DIVIDER) {
      plotCounter = 0;
      
      float curDiff = 0.0f, curSE = 0.0f, curMSE = 0.0f;
      if (xDataMutex != NULL && xSemaphoreTake(xDataMutex, 0) == pdTRUE) {
        curDiff = g_stats.diffPPG_BLE;
        curSE   = g_stats.sePPG_BLE;
        curMSE  = g_stats.msePPG_BLE;
        xSemaphoreGive(xDataMutex);
      } else {
        curDiff = g_stats.diffPPG_BLE;
        curSE   = g_stats.sePPG_BLE;
        curMSE  = g_stats.msePPG_BLE;
      }
      
      Serial.print("PPG_Raw:");
      Serial.print(rawPPG);
      Serial.print("\tPPG_Filt:");
      Serial.print(filteredPPG + ADC_TARGET_OFFSET);
      Serial.print("\tBPM_PPG:");
      Serial.print(g_bpmPPG);
      Serial.print("\tBPM_BLE:");
      Serial.print(g_bpmBLE);
      Serial.print("\tDiff:");
      Serial.print(curDiff);
      Serial.print("\tSE:");
      Serial.print(curSE);
      Serial.print("\tMSE:");
      Serial.println(curMSE);
      
      Serial.printf("CSV,%lu,%.2f,%.2f,%.2f,%.2f,%.2f\n", nowMs, g_bpmPPG, g_bpmBLE, curDiff, curSE, curMSE);
    }
    
    PPGSample sample = { rawPPG, filteredPPG };
    if (xSampleQueue != NULL) {
      xQueueSend(xSampleQueue, &sample, 0);
    }
  }
}

void vTaskBufferUpdate(void *pvParameters) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  for (;;) {
    PPGSample sample;
    if (xSampleQueue != NULL) {
      while (xQueueReceive(xSampleQueue, &sample, 0) == pdTRUE) {
      }
    }
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(20)); 
  }
}

void vTaskBLEServer(void *pvParameters) {
  BLEDevice::init(NOME_BLE);
  BLEDevice::setMTU(MTU_ALVO);
  BLEServer *servidor = BLEDevice::createServer();
  servidor->setCallbacks(new ServidorCallbacks());
  BLEService *servico = servidor->createService(UUID_SERVICO);
  BLECharacteristic *rx = servico->createCharacteristic(
                            UUID_RX,
                            BLECharacteristic::PROPERTY_WRITE | 
                            BLECharacteristic::PROPERTY_WRITE_NR
                          );
  rx->setCallbacks(new RxCallbacks());
  servico->start();
  
  BLEAdvertising *anuncio = BLEDevice::getAdvertising();
  anuncio->addServiceUUID(UUID_SERVICO);
  anuncio->setScanResponse(true);
  anuncio->setMinPreferred(0x06); 
  anuncio->setMinPreferred(0x12);
  BLEDevice::startAdvertising();
  
  for (;;) {
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

void vTaskEncoderUI(void *pvParameters) {
  EncoderEvent event;
  for (;;) {
    if (xQueueReceive(xUIEventQueue, &event, portMAX_DELAY) == pdTRUE) {
      if (event == EVENT_NEXT_SCREEN || event == EVENT_CLICK) {
        g_currentMenuState = static_cast<UIMenuState>((g_currentMenuState + 1) % 3);
      } else if (event == EVENT_PREV_SCREEN) {
        g_currentMenuState = static_cast<UIMenuState>((g_currentMenuState + 2) % 3);
      }
    }
  }
}

void vTaskDisplayRefresh(void *pvParameters) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  char textBuf[32];
  
  for (;;) {
    if (!g_oledOk) {
      vTaskDelay(pdMS_TO_TICKS(500));
      continue;
    }

    float bpmP = 0, bpmB = 0;
    BiomedicalStats statsLocal;
    if (xDataMutex != NULL && xSemaphoreTake(xDataMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
      bpmP = g_bpmPPG;
      bpmB = g_bpmBLE;
      statsLocal = g_stats;
      xSemaphoreGive(xDataMutex);
    } else {
      bpmP = g_bpmPPG;
      bpmB = g_bpmBLE;
      statsLocal = g_stats;
    }
    
    display.clearDisplay();
    display.setTextWrap(false);
    display.setTextColor(TEXT_COLOR_DEFAULT);
    
    if (g_currentMenuState == UI_STATE_MONITOR) {
      display.setCursor(6, 0);
      display.setTextSize(FONT_SIZE_HEADER);
      display.print(STR_TITLE_MONITOR);
      display.drawFastHLine(0, 9, 128, TEXT_COLOR_DEFAULT);
      
      // PPG Local
      display.setCursor(0, 13);
      display.setTextSize(FONT_SIZE_LABEL);
      display.print(STR_LABEL_PPG_LOC);
      
      display.setCursor(20, 23);
      display.setTextSize(FONT_SIZE_VALUE);
      if (bpmP >= 30.0f) {
        snprintf(textBuf, sizeof(textBuf), "%3d", (int)bpmP);
        display.print(textBuf);
        display.setCursor(65, 27);
        display.setTextSize(FONT_SIZE_UNIT);
        display.print(STR_UNIT_BPM);
      } else {
        display.print("---");
      }
      
      // PPG Remoto (BLE)
      display.setCursor(0, 41);
      display.setTextSize(FONT_SIZE_LABEL);
      display.print(STR_LABEL_PPG_BLE);
      
      display.setCursor(20, 50);
      display.setTextSize(FONT_SIZE_VALUE);
      if (bpmB >= 30.0f) {
        snprintf(textBuf, sizeof(textBuf), "%3d", (int)bpmB);
        display.print(textBuf);
        display.setCursor(65, 54);
        display.setTextSize(FONT_SIZE_UNIT);
        display.print(STR_UNIT_BPM);
      } else {
        display.print("---");
      }
    } 
    else if (g_currentMenuState == UI_STATE_BPM_COMPARE) {
      display.setCursor(6, 0);
      display.setTextSize(FONT_SIZE_HEADER);
      display.print(STR_TITLE_STATS1);
      display.drawFastHLine(0, 9, 128, TEXT_COLOR_DEFAULT);
      
      display.setCursor(0, 13);
      display.setTextSize(1);
      display.print("Media PPGs:");
      
      display.setCursor(0, 23);
      display.setTextSize(2);
      if (statsLocal.meanPPG >= 30.0f) {
        snprintf(textBuf, sizeof(textBuf), "%4.1f", statsLocal.meanPPG);
        display.print(textBuf);
        display.setTextSize(1);
        display.setCursor(64, 27);
        display.print(" BPM");
      } else {
        display.print(" ---");
      }
      
      display.setCursor(0, 41);
      display.setTextSize(1);
      display.print("Dif |PPG - BLE|:");
      
      display.setCursor(0, 50);
      display.setTextSize(2);
      if (bpmP >= 30.0f && bpmB >= 30.0f) {
        snprintf(textBuf, sizeof(textBuf), "%4.1f", statsLocal.diffPPG_BLE);
        display.print(textBuf);
        display.setTextSize(1);
        display.setCursor(64, 54);
        display.print(" BPM");
      } else {
        display.print(" ---");
      }
    }
    else if (g_currentMenuState == UI_STATE_MSE) {
      display.setCursor(4, 0);
      display.setTextSize(FONT_SIZE_HEADER);
      display.print(STR_TITLE_STATS2);
      display.drawFastHLine(0, 9, 128, TEXT_COLOR_DEFAULT);
      
      display.setCursor(0, 13);
      display.setTextSize(1);
      display.print("MSE Acumulado:");
      
      display.setCursor(0, 23);
      display.setTextSize(2);
      if (bpmP >= 30.0f && bpmB >= 30.0f && statsLocal.msePPG_BLE > 0.0f) {
        snprintf(textBuf, sizeof(textBuf), "%6.2f", statsLocal.msePPG_BLE);
        display.print(textBuf);
      } else {
        display.print("  ---");
      }
      
      display.setCursor(0, 41);
      display.setTextSize(1);
      display.print("Erro Quad. (SE):");
      
      display.setCursor(0, 50);
      display.setTextSize(2);
      if (bpmP >= 30.0f && bpmB >= 30.0f && statsLocal.sePPG_BLE > 0.0f) {
        snprintf(textBuf, sizeof(textBuf), "%6.2f", statsLocal.sePPG_BLE);
        display.print(textBuf);
      } else {
        display.print("  ---");
      }
    }
    
    display.display();
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(50));
  }
}

void setup() {
  Serial.begin(115200); 
  
  pinMode(PIN_PPG_ADC, INPUT);
  analogSetPinAttenuation(PIN_PPG_ADC, ADC_11db);
  
  pinMode(PIN_LED_CTRL, OUTPUT);
  digitalWrite(PIN_LED_CTRL, HIGH);
  
  pinMode(PIN_ENCODER_CLK, INPUT_PULLUP);
  pinMode(PIN_ENCODER_DT, INPUT_PULLUP);
  pinMode(PIN_ENCODER_SW, INPUT_PULLUP);
  
  ledcAttach(PIN_OFFSET_PWM, PWM_FREQ_HZ, PWM_RESOLUTION_BITS);
  ledcWrite(PIN_OFFSET_PWM, (uint32_t)PWM_INITIAL_DUTY);
  
  attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_CLK), onEncoderClkISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_SW), onEncoderSwISR, FALLING);
  
  Wire.begin(PIN_OLED_SDA, PIN_OLED_SCL);
  
  g_oledOk = display.begin(SSD1306_SWITCHCAPVCC, OLED_I2C_ADDRESS);
  if (g_oledOk) {
    display.clearDisplay();
    display.setTextSize(FONT_SIZE_HEADER);
    display.setTextColor(TEXT_COLOR_DEFAULT);
    display.setCursor(15, 25);
    display.println(STR_BOOT_TITLE);
    display.setCursor(15, 40);
    display.println(STR_BOOT_SUBTITLE);
    display.display();
    delay(500);
  }
  
  xDataMutex = xSemaphoreCreateMutex();
  xSampleQueue = xQueueCreate(20, sizeof(PPGSample));
  xUIEventQueue = xQueueCreate(10, sizeof(EncoderEvent));
  
  xTaskCreatePinnedToCore(vTaskPPGSampling,     "Sampling_Task",    4096, NULL, 5, &xSamplingTaskHandle, 1);
  xTaskCreatePinnedToCore(vTaskBufferUpdate,     "Buffer_Task",      3072, NULL, 3, NULL, 1);
  xTaskCreatePinnedToCore(vTaskBLEServer,        "BLE_Server_Task",  4096, NULL, 2, NULL, 0);
  xTaskCreatePinnedToCore(vTaskEncoderUI,        "Encoder_UI_Task",  3072, NULL, 2, NULL, 0);
  xTaskCreatePinnedToCore(vTaskDisplayRefresh,   "Display_OLED_Task", 4096, NULL, 1, NULL, 0);
  
  g_samplingTimer = timerBegin(1000000);
  timerAttachInterrupt(g_samplingTimer, &onSamplingTimerISR);
  timerAlarm(g_samplingTimer, SAMPLING_PERIOD_US, true, 0);
}

void loop() {
  vTaskDelete(NULL);
}