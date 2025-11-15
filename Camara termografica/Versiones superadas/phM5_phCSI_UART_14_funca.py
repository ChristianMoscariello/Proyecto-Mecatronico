# ----------------------------------------------------------------
# LIBRERÍAS
# ----------------------------------------------------------------
import json
import time
from datetime import datetime
import os
import serial
from picamera2 import Picamera2
import numpy as np
import matplotlib
matplotlib.use('Agg') # Backend no interactivo
import matplotlib.pyplot as plt
import matplotlib.patheffects as PathEffects
import matplotlib.patches as patches
import seaborn as sns
from scipy.ndimage import label, median_filter, find_objects

# ----------------------------------------------------------------
# CONFIGURACIÓN GENERAL
# ----------------------------------------------------------------
# --- Conexiones Serie ---
# Conexión para RECIBIR datos del M5StickC (vía USB)
PUERTO_M5 = '/dev/ttyUSB0'
VELOCIDAD_M5 = 115200 # La velocidad para USB suele ser alta

# Conexión para ENVIAR alertas al ESP32 (vía GPIO)
PUERTO_ESP32 = '/dev/serial0'
VELOCIDAD_ESP32 = 9600

# --- Cámara CSI ---
RESOLUCION_CAMARA = (1920, 1080) # Resolución Full HD

# ----------------------------------------------------------------
# PARÁMETROS DE DETECCIÓN
# ----------------------------------------------------------------
# Persona
TEMP_PERSONA_MIN = 25.0
TEMP_PERSONA_MAX = 38.0
AREA_PERSONA_MIN = 3
AREA_PERSONA_MAX = 40

# Incendio
TEMP_INCENDIO_MIN = 50.0
AREA_INCENDIO_MIN = 2

# ----------------------------------------------------------------
# GESTIÓN DE CARPETAS DE SALIDA
# ----------------------------------------------------------------
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
CARPETA_SALIDA_TERMICA = os.path.join(SCRIPT_DIR, 'imagenes_termicas_analizadas')
CARPETA_SALIDA_CSI = os.path.join(SCRIPT_DIR, 'imagenes_visual_deteccion')

if not os.path.exists(CARPETA_SALIDA_TERMICA):
    os.makedirs(CARPETA_SALIDA_TERMICA)
    print(f"Carpeta '{CARPETA_SALIDA_TERMICA}' creada.")

if not os.path.exists(CARPETA_SALIDA_CSI):
    os.makedirs(CARPETA_SALIDA_CSI)
    print(f"Carpeta '{CARPETA_SALIDA_CSI}' creada.")

# ----------------------------------------------------------------
# FUNCIÓN PARA ENVIAR ALERTAS AL ESP32
# ----------------------------------------------------------------
def enviar_alerta_esp32(serial_conn, tipo_alerta):
    """Construye y envía un JSON de alerta al ESP32."""
    try:
        mensaje_json = f'{{"t":"{tipo_alerta}"}}\n'
        serial_conn.write(mensaje_json.encode('utf-8'))
        print(f"📨 Mensaje enviado a ESP32: {tipo_alerta}")
    except Exception as e:
        print(f"❌ Error al enviar mensaje a ESP32: {e}")

# ----------------------------------------------------------------
# FUNCIÓN DE ANÁLISIS DE IMAGEN TÉRMICA
# ----------------------------------------------------------------
def analizar_frame(frame_array):
    """Analiza un array de temperaturas y detecta personas o focos de incendio."""
    matriz_termica = frame_array.reshape(24, 32)
    detecciones = []
    bboxes = []

    # === Detección de Personas ===
    temp_promedio_fondo = np.mean(matriz_termica[matriz_termica < TEMP_PERSONA_MIN])
    umbral_persona = max(TEMP_PERSONA_MIN, temp_promedio_fondo + 5) # Umbral dinámico con mínimo
    mascara_persona = (matriz_termica > umbral_persona) & (matriz_termica <= TEMP_PERSONA_MAX)
    labels_persona, num_features_persona = label(mascara_persona)

    if num_features_persona > 0:
        slice_objects_persona = find_objects(labels_persona)
        for i in range(1, num_features_persona + 1):
            blob = matriz_termica[labels_persona == i]
            area = len(blob)
            if AREA_PERSONA_MIN <= area <= AREA_PERSONA_MAX:
                detecciones.append("Persona")
                slice_y, slice_x = slice_objects_persona[i-1]
                width = slice_x.stop - slice_x.start
                height = slice_y.stop - slice_y.start
                x_start_flipped = matriz_termica.shape[1] - slice_x.stop
                bboxes.append((x_start_flipped, slice_y.start, width, height))

    # === Detección de Incendios ===
    mascara_incendio = matriz_termica > TEMP_INCENDIO_MIN
    labels_incendio, num_features_incendio = label(mascara_incendio)
    if num_features_incendio > 0:
        slice_objects_incendio = find_objects(labels_incendio)
        for i in range(1, num_features_incendio + 1):
            area = np.sum(labels_incendio == i)
            if area >= AREA_INCENDIO_MIN:
                detecciones.append("Foco de Incendio")
                slice_y, slice_x = slice_objects_incendio[i-1]
                width = slice_x.stop - slice_x.start
                height = slice_y.stop - slice_y.start
                x_start_flipped = matriz_termica.shape[1] - slice_x.stop
                bboxes.append((x_start_flipped, slice_y.start, width, height))
            
    return list(set(detecciones)), bboxes

# ----------------------------------------------------------------
# INICIALIZACIÓN DE HARDWARE
# ----------------------------------------------------------------
picam2 = None
m5_serial = None
esp32_serial = None
print("Iniciando sistema de análisis...")

try:
    print("Inicializando cámara CSI...")
    picam2 = Picamera2()
    camera_config = picam2.create_still_configuration(main={"size": RESOLUCION_CAMARA})
    picam2.configure(camera_config)
    picam2.start()
    time.sleep(2)
    print("✅ Cámara CSI inicializada.")
except Exception as e:
    print(f"❌ ERROR: No se pudo inicializar la cámara CSI: {e}")
    picam2 = None

# ----------------------------------------------------------------
# BUCLE PRINCIPAL DE EJECUCIÓN
# ----------------------------------------------------------------
try:
    print(f"Conectando al M5StickC en {PUERTO_M5}...")
    m5_serial = serial.Serial(PUERTO_M5, VELOCIDAD_M5, timeout=2)
    print("✅ Conexión con M5StickC establecida.")

    print(f"Conectando al ESP32 en {PUERTO_ESP32}...")
    esp32_serial = serial.Serial(PUERTO_ESP32, VELOCIDAD_ESP32)
    print("✅ Conexión con ESP32 establecida.")
    
    time.sleep(2)
    print("\n--- Sistema Activo: Esperando datos del sensor térmico ---")

    while True:
        linea_bytes = m5_serial.readline()
        if not linea_bytes:
            continue
        
        linea_str = linea_bytes.decode('utf-8', errors='ignore').strip()

        if not linea_str.startswith('{'):
            if linea_str: print(f"MSG DEL M5: {linea_str}")
            continue

        try:
            datos = json.loads(linea_str)
            temperaturas = datos.get('temperaturas')

            if temperaturas and len(temperaturas) == 768:
                array_temperaturas = np.array(temperaturas)
                mapa_termico_filtrado = median_filter(array_temperaturas.reshape(24, 32), size=3)
                resultados_analisis, bboxes_detectados = analizar_frame(mapa_termico_filtrado.flatten())
                mapa_termico_visual = np.fliplr(mapa_termico_filtrado)
                
                now = datetime.now()
                timestamp = now.strftime("%y%m%d_%H%M%S")
                nombre_archivo_termico = f"mapa_{timestamp}.png"
                ruta_completa_termica = os.path.join(CARPETA_SALIDA_TERMICA, nombre_archivo_termico)
                
                fig, ax = plt.subplots(figsize=(8, 6))
                sns.heatmap(mapa_termico_visual, ax=ax, cmap="inferno", cbar_kws={'label': 'Temperatura (°C)'})
                ax.set_title(f"Mapa de Calor - {now.strftime('%d/%m/%Y %H:%M:%S')}")
                ax.set_xticks([]); ax.set_yticks([])

                for bbox in bboxes_detectados:
                    x, y, w, h = bbox
                    rect = patches.Rectangle((x, y), w, h, linewidth=2, edgecolor='lime', facecolor='none')
                    ax.add_patch(rect)

                texto_deteccion = "Sin Detecciones"
                color_texto = "green"
                if resultados_analisis:
                    texto_deteccion = "¡ALERTA!: " + ", ".join(resultados_analisis)
                    color_texto = "red"
                    
                    if "Persona" in resultados_analisis:
                        enviar_alerta_esp32(esp32_serial, "PERSON")
                    
                    if "Foco de Incendio" in resultados_analisis:
                        enviar_alerta_esp32(esp32_serial, "FIRE")
                    
                    if picam2:
                        nombre_archivo_visual = f"visual_{timestamp}.jpg"
                        ruta_completa_visual = os.path.join(CARPETA_SALIDA_CSI, nombre_archivo_visual)
                        try:
                            picam2.capture_file(ruta_completa_visual) 
                            print(f"📸 Foto visual capturada: {nombre_archivo_visual}")
                        except Exception as e:
                            print(f"❌ ERROR al tomar foto: {e}")
                            
                txt = fig.text(0.5, 0.05, texto_deteccion, ha='center', va='bottom', fontsize=14, color=color_texto, weight='bold')
                txt.set_path_effects([PathEffects.withStroke(linewidth=3, foreground='black')])

                plt.savefig(ruta_completa_termica, bbox_inches='tight')
                plt.close(fig)

                print(f"Análisis completado: {nombre_archivo_termico} | Estado: {texto_deteccion}")

        except json.JSONDecodeError:
            continue
        except Exception as e:
            print(f"❌ Ocurrió un error en el bucle principal: {e}")

except serial.SerialException as e:
    print(f"❌ ERROR DE CONEXIÓN SERIAL: {e}")
    print("Verifica las conexiones y que los puertos {PUERTO_M5} y {PUERTO_ESP32} sean correctos.")
except KeyboardInterrupt:
    print("\nPrograma detenido por el usuario.")
finally:
    if picam2 and picam2.started:
        print("Deteniendo cámara CSI...")
        picam2.stop()
    if m5_serial and m5_serial.is_open:
        m5_serial.close()
    if esp32_serial and esp32_serial.is_open:
        esp32_serial.close()
    print("Sistema finalizado.")