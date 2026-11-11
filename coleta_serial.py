import serial
import csv
import time
import sys

# ============================================================================================
# CONFIGURAÇÕES DA PORTA SERIAL E ARQUIVO
# ============================================================================================
PORTA_SERIAL = 'COM3'        
BAUDRATE     = 115200         
NOME_ARQUIVO = 'coleta_biomedica.csv'

# ============================================================================================
# CABEÇALHO DO ARQUIVO CSV DE SAÍDA
# ============================================================================================
HEADERS = [
    'Timestamp_ms', 
    'BPM_PPG', 
    'BPM_BLE', 
    'Diff_BPM',
    'SE_BPM2',              
    'MSE_BPM2'              
]

def main():
    print("=" * 70)
    print("INICIANDO MONITORAMENTO E COLETA DE DADOS SERIAL ESP32-S3 (PPG)")
    print(f"Porta: {PORTA_SERIAL} | Baudrate: {BAUDRATE}")
    print(f"Arquivo de Saída: {NOME_ARQUIVO}")
    print("Pressione Ctrl+C para encerrar a coleta.")
    print("=" * 70)

    # Conexão com a Porta Serial
    try:
        ser = serial.Serial(PORTA_SERIAL, BAUDRATE, timeout=1)
        time.sleep(2)  # Aguarda estabilização do DTR/RTS
        ser.flushInput()
    except Exception as e:
        print(f"[ERRO] Não foi possível abrir a porta {PORTA_SERIAL}: {e}")
        print("Verifique se a ESP32 está conectada e se a porta Serial está correta.")
        sys.exit(1)

    count_ble = 0
    sum_se_ble = 0.0

    # Abertura do Arquivo CSV
    try:
        with open(NOME_ARQUIVO, mode='w', newline='', encoding='utf-8') as file:
            writer = csv.writer(file)
            writer.writerow(HEADERS)
            file.flush()

            while True:
                if ser.in_waiting > 0:
                    try:
                        linha_bruta = ser.readline().decode('utf-8', errors='ignore').strip()
                    except Exception:
                        continue

                    
                    if linha_bruta.startswith("CSV,"):
                        partes = linha_bruta.split(',')
                        if len(partes) >= 7:
                            try:
                                timestamp_ms = int(partes[1])
                                bpm_ppg      = float(partes[2])
                                bpm_ble      = float(partes[3])
                                diff_bpm     = float(partes[4])
                                se_val       = float(partes[5])
                                mse_val      = float(partes[6])

                                if bpm_ppg >= 30.0 and bpm_ble >= 30.0:
                                    sum_se_ble += se_val
                                    count_ble += 1

                                row = [
                                    timestamp_ms,
                                    f"{bpm_ppg:.2f}",
                                    f"{bpm_ble:.2f}",
                                    f"{diff_bpm:.2f}",
                                    f"{se_val:.2f}",
                                    f"{mse_val:.2f}"
                                ]
                                writer.writerow(row)
                                file.flush()

                                print(f"[{timestamp_ms:8d} ms] PPG: {bpm_ppg:5.1f} | BLE: {bpm_ble:5.1f} | "
                                      f"Dif: {diff_bpm:5.1f} | SE: {se_val:7.2f} | MSE: {mse_val:7.2f}")
                            except ValueError:
                                pass

    except KeyboardInterrupt:
        print("\n" + "=" * 70)
        print("COLETA ENCERRADA PELO USUÁRIO.")
        if count_ble > 0:
            print(f"Total de amostras pareadas: {count_ble}")
            print(f"MSE Médio Global (PPG - BLE): {sum_se_ble / count_ble:.2f} BPM²")
        print(f"Dados salvos com sucesso em '{NOME_ARQUIVO}'.")
        print("=" * 70)

    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()

if __name__ == '__main__':
    main()
