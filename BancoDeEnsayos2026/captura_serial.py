import serial
import csv
import time
from datetime import datetime
import sys

# ================= CONFIGURACIÓN =================
# Cambia 'COM3' por el puerto donde esté conectado tu ESP32/Arduino.
# En Windows suele ser 'COM3', 'COM4', etc.
# En Linux/Mac suele ser '/dev/ttyUSB0' o '/dev/cu.usbserial...'
PUERTO_SERIE = 'COM3' 
BAUD_RATE = 115200
ARCHIVO_CSV = 'datos_medicion.csv'
SEPARADOR = ';' # Usa ';' si tu Excel en español junta todo en una columna
# =================================================

def iniciar_captura():
    try:
        # Iniciamos la conexión serie
        ser = serial.Serial(PUERTO_SERIE, BAUD_RATE, timeout=1)
        print(f"Conectado exitosamente al puerto {PUERTO_SERIE} a {BAUD_RATE} baudios.")
    except Exception as e:
        print(f"Error al conectar con el puerto {PUERTO_SERIE}: {e}")
        sys.exit(1)

    # Variables para guardar el último dato recibido
    # (Como la fuerza llega más rápido, repetiremos la última temperatura conocida)
    ultima_fuerza = ""
    ultima_temp = ""

    # Abrimos el archivo CSV en modo "append" (añadir)
    with open(ARCHIVO_CSV, mode='a', newline='') as archivo:
        escritor = csv.writer(archivo, delimiter=SEPARADOR)
        
        # Escribimos las cabeceras si el archivo está vacío
        # (Si vas a hacer varias pruebas seguidas, puedes comentar esta línea)
        escritor.writerow(["Timestamp", "Fuerza (Kg)", "Temperatura (C)"])

        print(f"Guardando datos en '{ARCHIVO_CSV}'...")
        print("Presiona Ctrl+C en esta consola para detener la captura.\n")

        try:
            while True:
                if ser.in_waiting > 0:
                    linea_cruda = ser.readline()
                    
                    try:
                        # Decodificamos de bytes a texto y quitamos saltos de línea (\r\n)
                        linea = linea_cruda.decode('utf-8').strip()
                    except UnicodeDecodeError:
                        continue # Ignorar basura o bytes corruptos en la línea

                    if not linea:
                        continue

                    # Generamos una marca de tiempo con milisegundos
                    timestamp = datetime.now().strftime("%H:%M:%S")
                    guardar_fila = False

                    # Analizamos qué tipo de dato nos ha llegado
                    if linea.startswith("Fuerza(Kg):"):
                        try:
                            ultima_fuerza = float(linea.split(":")[1])
                            guardar_fila = True
                        except ValueError:
                            pass

                    elif linea.startswith("Temperatura:"):
                        try:
                            ultima_temp = float(linea.split(":")[1])
                            guardar_fila = True
                        except ValueError:
                            pass

                    else:
                        # Si no es ni Fuerza ni Temperatura, probablemente es un mensaje de FAULT o debug.
                        # Lo imprimimos en pantalla pero NO lo metemos al CSV.
                        print(f"[{timestamp}] MENSAJE DEL MICRO: {linea}")

                    # Si hemos actualizado algún valor, guardamos la fila entera
                    if guardar_fila:
                        escritor.writerow([timestamp, ultima_fuerza, ultima_temp])
                        archivo.flush() # Forzamos el guardado en disco por si se corta la luz

        except KeyboardInterrupt:
            print("\nCaptura detenida por el usuario (Ctrl+C).")
        finally:
            ser.close()
            print("Puerto serie cerrado de forma segura.")

if __name__ == '__main__':
    iniciar_captura()