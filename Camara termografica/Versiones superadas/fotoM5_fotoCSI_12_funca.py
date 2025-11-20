# ----------------------------------------------------------------
# LIBRERÍAS
# ----------------------------------------------------------------
# --- LIBRERÍAS ESTÁNDAR ---
import json
import time
from datetime import datetime
import os

# --- CONTROL DE HARDWARE ---
import serial
from picamera2 import Picamera2
from libcamera import controls

# --- ANÁLISIS Y VISUALIZACIÓN DE DATOS ---
import numpy as np
import matplotlib
matplotlib.use('Agg') # Backend no interactivo para evitar problemas de GUI
import matplotlib.pyplot as plt
import matplotlib.patheffects as PathEffects
import matplotlib.patches as patches
import seaborn as sns
from scipy.ndimage import label, median_filter, find_objects

# ----------------------------------------------------------------
# CONFIGURACIÓN GENERAL
# ----------------------------------------------------------------
# Puerto serie de la Raspberry Pi (reemplazar si es necesario)
PUERTO_SERIE = '/dev/ttyUSB0'
VELOCIDAD = 115200

# Cámara CSI
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
# GESTIÓN DE CARPETAS DE SALIDA (ORDEN CORREGIDO)
# ----------------------------------------------------------------
# 1. Obtenemos la ruta del script PRIMERO.
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))

# 2. Usamos SCRIPT_DIR para crear las demás rutas.
CARPETA_SALIDA_TERMICA = os.path.join(SCRIPT_DIR, 'imagenes_termicas_analizadas')
CARPETA_SALIDA_CSI = os.path.join(SCRIPT_DIR, 'imagenes_visual_deteccion')

# 3. Creamos las carpetas si no existen.
if not os.path.exists(CARPETA_SALIDA_TERMICA):
    os.makedirs(CARPETA_SALIDA_TERMICA)
    print(f"Carpeta '{CARPETA_SALIDA_TERMICA}' creada.")

if not os.path.exists(CARPETA_SALIDA_CSI):
    os.makedirs(CARPETA_SALIDA_CSI)
    print(f"Carpeta '{CARPETA_SALIDA_CSI}' creada para imágenes visuales.")

# ----------------------------------------------------------------
# FUNCIÓN DE ANÁLISIS DE IMAGEN
# ----------------------------------------------------------------
def analizar_frame(frame_array):
    """
    Analiza un array de 768 temperaturas y detecta personas o focos de incendio.
    Devuelve una lista con los tipos de detección encontrados y las cajas delimitadoras.
    """
    matriz_termica = frame_array.reshape(24, 32)
    detecciones = []
    bboxes = []

    # === Detección de Personas (Umbral Dinámico) ===
    temp_promedio = np.mean(matriz_termica)
    temp_desv = np.std(matriz_termica)
    umbral_persona = temp_promedio + 2 * temp_desv
    mascara_persona = matriz_termica > umbral_persona
    labels_persona, num_features_persona = label(mascara_persona)
    slice_objects_persona = find_objects(labels_persona)

    for i in range(1, num_features_persona + 1):
        blob = matriz_termica[labels_persona == i]
        area = len(blob)
        temp_promedio_blob = np.mean(blob)

        if (AREA_PERSONA_MIN <= area <= AREA_PERSONA_MAX and
            TEMP_PERSONA_MIN <= temp_promedio_blob <= TEMP_PERSONA_MAX):
            detecciones.append("Persona")
            
            slice_y, slice_x = slice_objects_persona[i-1]
            y_start, y_end = slice_y.start, slice_y.stop
            x_start, x_end = slice_x.start, slice_x.stop
            width = x_end - x_start
            height = y_end - y_start
            
            # Ajuste de coordenada 'x' por el uso de np.fliplr en la visualización
            x_start_flipped = matriz_termica.shape[1] - x_end
            bboxes.append((x_start_flipped, y_start, width, height))
            # Se eliminó 'break' para detectar MÚLTIPLES personas

    # === Detección de Incendios (Umbral Fijo) ===
    mascara_incendio = matriz_termica > TEMP_INCENDIO_MIN
    labels_incendio, num_features_incendio = label(mascara_incendio)
    slice_objects_incendio = find_objects(labels_incendio)

    for i in range(1, num_features_incendio + 1):
        blob = matriz_termica[labels_incendio == i]
        area = len(blob)
        if area >= AREA_INCENDIO_MIN:
            detecciones.append("Foco de Incendio")
            
            slice_y, slice_x = slice_objects_incendio[i-1]
            y_start, y_end = slice_y.start, slice_y.stop
            x_start, x_end = slice_x.start, slice_x.stop
            width = x_end - x_start
            height = y_end - y_start

            x_start_flipped = matriz_termica.shape[1] - x_end
            bboxes.append((x_start_flipped, y_start, width, height))
            # Se eliminó 'break' para detectar MÚLTIPLES focos de incendio
            
    return list(set(detecciones)), bboxes

# ----------------------------------------------------------------
# INICIALIZACIÓN DE HARDWARE
# ----------------------------------------------------------------
picam2 = None  # Inicializar variable fuera del try
print("Iniciando sistema de análisis...")

# Inicializar Cámara CSI
try:
    print("Inicializando cámara CSI...")
    picam2 = Picamera2()
    camera_config = picam2.create_still_configuration(main={"size": RESOLUCION_CAMARA})
    picam2.configure(camera_config)
    picam2.start()
    time.sleep(2) # Dar tiempo a la cámara para iniciar
    print("Cámara CSI inicializada correctamente.")
except Exception as e:
    print(f"❌ ERROR: No se pudo inicializar la cámara CSI: {e}")
    picam2 = None # Asegurarse de que sea None si falla

# ----------------------------------------------------------------
# BUCLE PRINCIPAL DE EJECUCIÓN
# ----------------------------------------------------------------
try:
    print(f"Intentando conectar al puerto {PUERTO_SERIE}...")
    arduino = serial.Serial(PUERTO_SERIE, VELOCIDAD, timeout=2)
    time.sleep(2)
    print("¡Conexión serial establecida! Esperando datos...")

    while True:
        linea_bytes = arduino.readline()
        if not linea_bytes:
            continue
        
        linea_str = linea_bytes.decode('utf-8').strip()

        if not linea_str.startswith('{'):
            if linea_str:
                print(f"MSG DEL M5: {linea_str}")
            continue

        try:
            datos = json.loads(linea_str)
            temperaturas = datos.get('temperaturas')

            if temperaturas and len(temperaturas) == 768:
                # 1. PROCESAR DATOS TÉRMICOS
                array_temperaturas = np.array(temperaturas)
                mapa_termico_filtrado = median_filter(array_temperaturas.reshape(24, 32), size=3)
                resultados_analisis, bboxes_detectados = analizar_frame(mapa_termico_filtrado.flatten())
                mapa_termico_visual = np.fliplr(mapa_termico_filtrado)
                
                # 2. GENERAR TIMESTAMPS Y NOMBRES DE ARCHIVO
                now = datetime.now()
                timestamp = now.strftime("%y%m%d_%H%M%S")
                nombre_archivo_termico = f"mapa_{timestamp}.png"
                ruta_completa_termica = os.path.join(CARPETA_SALIDA_TERMICA, nombre_archivo_termico)
                
                # 3. CREAR LA FIGURA DEL MAPA DE CALOR
                fig, ax = plt.subplots(figsize=(8, 6))
                sns.heatmap(mapa_termico_visual, ax=ax, cmap="inferno", cbar_kws={'label': 'Temperatura (°C)'})
                ax.set_title(f"Mapa de Calor - {now.strftime('%d/%m/%Y %H:%M:%S')}")
                ax.set_xticks([])
                ax.set_yticks([])

                # 4. DIBUJAR RECUADROS PARA TODAS LAS DETECCIONES
                for bbox in bboxes_detectados:
                    x, y, width, height = bbox
                    rect = patches.Rectangle((x, y), width, height, 
                                             linewidth=2, edgecolor='lime', facecolor='none')
                    ax.add_patch(rect)

                # 5. LÓGICA DE DETECCIÓN Y CAPTURA DE IMAGEN VISUAL
                texto_deteccion = "Sin Detecciones"
                color_texto = "green"
                if resultados_analisis:
                    texto_deteccion = "¡ALERTA!: " + ", ".join(resultados_analisis)
                    color_texto = "red"
                    
                    if picam2: # Solo intentar capturar si la cámara se inicializó bien
                        nombre_archivo_visual = f"visual_{timestamp}.jpg"
                        ruta_completa_visual = os.path.join(CARPETA_SALIDA_CSI, nombre_archivo_visual)
                        try:
                            picam2.capture_file(ruta_completa_visual) 
                            print(f"📸 Foto visual capturada: {nombre_archivo_visual}")
                        except Exception as camera_e:
                            print(f"❌ ERROR al tomar foto con cámara CSI: {camera_e}")
                            
                # 6. AÑADIR TEXTO DE ESTADO A LA IMAGEN (CORREGIDO)
                # Esta sección ahora está fuera del 'if' para que siempre se ejecute
                txt = fig.text(0.5, 0.05, texto_deteccion, 
                               ha='center', va='bottom', fontsize=14, color=color_texto,
                               weight='bold')
                txt.set_path_effects([PathEffects.withStroke(linewidth=3, foreground='black')])

                # 7. GUARDAR Y CERRAR LA IMAGEN TÉRMICA
                plt.savefig(ruta_completa_termica, bbox_inches='tight')
                plt.close(fig)

                print(f"✅ Imagen térmica guardada: {nombre_archivo_termico} | Estado: {texto_deteccion}")

        except json.JSONDecodeError:
            # Ignorar errores de JSON malformado que a veces envía el serial
            continue
        except Exception as e:
            print(f"❌ Ocurrió un error al procesar la imagen: {e}")

except serial.SerialException as e:
    print(f"❌ ERROR DE CONEXIÓN: No se pudo abrir el puerto {PUERTO_SERIE}.")
    print("Asegúrate de que el M5StickC esté conectado y que el Monitor Serie de Arduino esté cerrado.")
except KeyboardInterrupt:
    print("\nPrograma detenido por el usuario.")
finally:
    # DETENER LA CÁMARA AL FINALIZAR EL PROGRAMA
    if picam2 and picam2.started:
        print("Deteniendo cámara CSI...")
        picam2.stop()
        print("Cámara CSI detenida.")
    print("Sistema finalizado.")