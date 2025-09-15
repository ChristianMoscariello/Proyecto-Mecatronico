import serial
import json
import time
import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns
from datetime import datetime
import os

# --- CONFIGURACIÓN ---
# ❗ IMPORTANTE: Reemplaza 'COM3' con el puerto COM correcto de tu M5StickC.
PUERTO_SERIE = 'COM8'
VELOCIDAD = 115200
# ---------------------

# --- Definición de la ruta absoluta para la carpeta de salida ---
# Obtiene la ruta del directorio donde se encuentra este script
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
# Une esa ruta con el nombre de la carpeta que queremos usar
CARPETA_SALIDA = os.path.join(SCRIPT_DIR, 'imagenes_termicas')
# ----------------------------------------------------------------

# Crear la carpeta de salida si no existe
if not os.path.exists(CARPETA_SALIDA):
    os.makedirs(CARPETA_SALIDA)
    print(f"Carpeta '{CARPETA_SALIDA}' creada/asegurada.")

print("Iniciando receptor de mapas de calor...")
print(f"Las imágenes se guardarán en: {CARPETA_SALIDA}")
print(f"Intentando conectar al puerto {PUERTO_SERIE}...")

try:
    # Se establece la conexión con el puerto serie
    arduino = serial.Serial(PUERTO_SERIE, VELOCIDAD, timeout=2)
    time.sleep(2) # Esperar 2 segundos para que se estabilice la conexión
    print("¡Conexión establecida! Esperando datos del M5StickC...")

    # Bucle infinito para leer datos constantemente
    while True:
        linea_bytes = arduino.readline()
        if not linea_bytes:
            continue
        
        linea_str = linea_bytes.decode('utf-8').strip()

        # Si la línea no es un JSON, podría ser un mensaje de estado del M5StickC
        if not linea_str.startswith('{'):
            if linea_str: # Solo imprimir si no está vacío
                 print(f"MENSAJE DEL M5StickC: {linea_str}")
            continue

        # Intentar procesar la línea como un JSON
        try:
            datos = json.loads(linea_str)
            temperaturas = datos.get('temperaturas')

            if temperaturas and len(temperaturas) == 768:
                # --- Proceso de creación de imagen ---
                # 1. Convertir la lista a un array de NumPy y darle forma de 24x32
                mapa_termico = np.reshape(np.array(temperaturas), (24, 32))
                
                # 2. Corregir la rotación de 180 grados
                mapa_termico = np.flipud(np.fliplr(mapa_termico))
                mapa_termico = np.flipud(mapa_termico)

                # 3. Generar el nombre del archivo con fecha y hora
                now = datetime.now()
                timestamp = now.strftime("%y%m%d_%H%M%S") # Formato AAMMDD_HHMMSS
                nombre_archivo = f"mapa_de_calor_{timestamp}.png"
                ruta_completa = os.path.join(CARPETA_SALIDA, nombre_archivo)
                
                # 4. Crear y guardar la figura del mapa de calor
                fig, ax = plt.subplots(figsize=(8, 6))
                sns.heatmap(mapa_termico, ax=ax, cmap="inferno", cbar_kws={'label': 'Temperatura (°C)'})
                ax.set_title(f"Mapa de Calor - {now.strftime('%d/%m/%Y %H:%M:%S')}")
                ax.set_xticks([]) # Ocultar ejes para una imagen más limpia
                ax.set_yticks([])

                plt.savefig(ruta_completa)
                plt.close(fig) # Importante para liberar memoria

                print(f"✅ Imagen guardada: {nombre_archivo}")

        except Exception as e:
            print(f"❌ Ocurrió un error al procesar la imagen: {e}")

except serial.SerialException as e:
    print(f"❌ ERROR DE CONEXIÓN: No se pudo abrir el puerto {PUERTO_SERIE}.")
    print("   Asegúrate de que el M5StickC esté conectado y que el Monitor Serie de Arduino esté cerrado.")