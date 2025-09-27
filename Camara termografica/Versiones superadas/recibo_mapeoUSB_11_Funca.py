# Importaciones y configuración (sin cambios)
# ...

import serial
import json
import time
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patheffects as PathEffects
import seaborn as sns
from datetime import datetime
import os
from scipy.ndimage import label, median_filter, find_objects
import matplotlib.patches as patches

# ... (El resto de la configuración inicial se mantiene igual)
# ----------------------------------------------------------------
# CONFIGURACIÓN
# ----------------------------------------------------------------
# ❗ IMPORTANTE: Reemplaza 'COM8' con el puerto COM correcto de tu M5StickC.
PUERTO_SERIE = 'COM8'
VELOCIDAD = 115200

# ----------------------------------------------------------------
# PARÁMETROS DE DETECCIÓN (Ajustados para vista cenital)
# ----------------------------------------------------------------
# Persona
TEMP_PERSONA_MIN = 25.0
TEMP_PERSONA_MAX = 38.0
AREA_PERSONA_MIN = 3   # en píxeles (ajustado para objetos pequeños)
AREA_PERSONA_MAX = 40  # en píxeles (ajustado para objetos pequeños)

# Incendio
TEMP_INCENDIO_MIN = 50.0
AREA_INCENDIO_MIN = 2   # en píxeles

# ----------------------------------------------------------------
# GESTIÓN DE CARPETA DE SALIDA
# ----------------------------------------------------------------
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
CARPETA_SALIDA = os.path.join(SCRIPT_DIR, 'imagenes_termicas_analizadas')

if not os.path.exists(CARPETA_SALIDA):
    os.makedirs(CARPETA_SALIDA)
    print(f"Carpeta '{CARPETA_SALIDA}' creada.")

# ----------------------------------------------------------------
# FUNCIÓN DE ANÁLISIS DE IMAGEN (CON LA CORRECCIÓN PARA fliplr)
# ----------------------------------------------------------------
def analizar_frame(frame_array):
    """
    Analiza un array de 768 temperaturas y detecta personas o focos de incendio.
    Devuelve una lista con los tipos de detección encontrados y las cajas delimitadoras.
    """
    matriz_termica = frame_array.reshape(24, 32)
    detecciones = []
    bboxes = [] # Lista para almacenar las cajas delimitadoras (x, y, ancho, alto)

    # === Detección de Personas (Umbral Dinámico) ===
    temp_promedio = np.mean(matriz_termica)
    temp_desv = np.std(matriz_termica)
    umbral_persona = temp_promedio + 2 * temp_desv
    mascara_persona = matriz_termica > umbral_persona
    labels_persona, num_features_persona = label(mascara_persona) # type: ignore
    slice_objects_persona = find_objects(labels_persona)

    for i in range(1, num_features_persona + 1):
        blob = matriz_termica[labels_persona == i] # type: ignore
        area = len(blob)
        temp_promedio_blob = np.mean(blob)

        if (AREA_PERSONA_MIN <= area <= AREA_PERSONA_MAX and
                TEMP_PERSONA_MIN <= temp_promedio_blob <= TEMP_PERSONA_MAX):
            detecciones.append("Persona")
            
            # Obtener bounding box para personas
            slice_y, slice_x = slice_objects_persona[i-1]
            y_start = slice_y.start
            x_start, x_end = slice_x.start, slice_x.stop
            
            width = x_end - x_start
            height = slice_y.stop - y_start
            
            # ⭐ AJUSTE CLAVE PARA np.fliplr ⭐
            # La coordenada 'y' no cambia. La 'x' se invierte.
            # La nueva 'x' es el ancho total de la matriz menos la 'x' final del objeto.
            x_start_flipped = matriz_termica.shape[1] - x_end
            
            bboxes.append((x_start_flipped, y_start, width, height))
            break

    # === Detección de Incendios (Umbral Fijo) ===
    mascara_incendio = matriz_termica > TEMP_INCENDIO_MIN
    labels_incendio, num_features_incendio = label(mascara_incendio) # type: ignore
    slice_objects_incendio = find_objects(labels_incendio)

    for i in range(1, num_features_incendio + 1):
        blob = matriz_termica[labels_incendio == i] # type: ignore
        area = len(blob)
        if area >= AREA_INCENDIO_MIN:
            detecciones.append("Foco de Incendio")
            
            # Obtener bounding box para incendios
            slice_y, slice_x = slice_objects_incendio[i-1]
            y_start = slice_y.start
            x_start, x_end = slice_x.start, slice_x.stop
            
            width = x_end - x_start
            height = slice_y.stop - y_start

            # ⭐ AJUSTE CLAVE PARA np.fliplr ⭐
            x_start_flipped = matriz_termica.shape[1] - x_end

            bboxes.append((x_start_flipped, y_start, width, height))
            break
            
    return list(set(detecciones)), bboxes

# ----------------------------------------------------------------
# BUCLE PRINCIPAL DE EJECUCIÓN (CON TU SOLUCIÓN)
# ----------------------------------------------------------------
print("Iniciando sistema de análisis de imágenes térmicas...")
print(f"Las imágenes se guardarán en: {CARPETA_SALIDA}")
print(f"Intentando conectar al puerto {PUERTO_SERIE}...")

try:
    arduino = serial.Serial(PUERTO_SERIE, VELOCIDAD, timeout=2)
    time.sleep(2)
    print("¡Conexión establecida! Esperando datos...")

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
                
                # --- PROCESO POR IMAGEN ---
                array_temperaturas = np.array(temperaturas)
                
                # 1. PREPARAR MATRIZ TÉRMICA ORIGINAL
                mapa_termico_original = np.reshape(array_temperaturas, (24, 32))

                # 2. APLICAR FILTRO DE MEDIANA PARA REDUCIR RUIDO
                mapa_termico_filtrado = median_filter(mapa_termico_original, size=3)
                
                # 3. ANALIZAR EL FRAME FILTRADO PARA BUSCAR OBJETIVOS Y BBOXES
                resultados_analisis, bboxes_detectados = analizar_frame(mapa_termico_filtrado.flatten())

                # 4. CORREGIR ORIENTACIÓN HORIZONTAL PARA VISUALIZACIÓN (TU SOLUCIÓN)
                mapa_termico_visual = np.fliplr(mapa_termico_filtrado)
                
                # 5. GENERAR NOMBRE DE ARCHIVO Y TÍTULO
                now = datetime.now()
                timestamp = now.strftime("%y%m%d_%H%M%S")
                nombre_archivo = f"mapa_{timestamp}.png"
                ruta_completa = os.path.join(CARPETA_SALIDA, nombre_archivo)
                
                # 6. CREAR LA FIGURA DEL MAPA DE CALOR
                fig, ax = plt.subplots(figsize=(8, 6))
                sns.heatmap(mapa_termico_visual, ax=ax, cmap="inferno", cbar_kws={'label': 'Temperatura (°C)'})
                ax.set_title(f"Mapa de Calor - {now.strftime('%d/%m/%Y %H:%M:%S')}")
                ax.set_xticks([])
                ax.set_yticks([])

                # 7. DIBUJAR RECUADROS SI HAY DETECCIONES
                for bbox in bboxes_detectados:
                    x, y, width, height = bbox
                    rect = patches.Rectangle((x, y), width, height, 
                                             linewidth=2, edgecolor='lime', facecolor='none')
                    ax.add_patch(rect)

                # 8. AGREGAR TEXTO DE DETECCIÓN A LA IMAGEN
                texto_deteccion = "Sin Detecciones"
                color_texto = "green"
                if resultados_analisis:
                    texto_deteccion = "¡ALERTA!: " + ", ".join(resultados_analisis)
                    color_texto = "red"

                txt = fig.text(0.5, 0.05, texto_deteccion, 
                               ha='center', va='bottom', fontsize=14, color=color_texto,
                               weight='bold')
                txt.set_path_effects([PathEffects.withStroke(linewidth=3, foreground='black')])

                # 9. GUARDAR Y CERRAR LA IMAGEN
                plt.savefig(ruta_completa, bbox_inches='tight')
                plt.close(fig)

                print(f"✅ Imagen guardada: {nombre_archivo} | Estado: {texto_deteccion}")

        except Exception as e:
            print(f"❌ Ocurrió un error al procesar la imagen: {e}")

except serial.SerialException as e:
    print(f"❌ ERROR DE CONEXIÓN: No se pudo abrir el puerto {PUERTO_SERIE}.")
    print("Asegúrate de que el M5StickC esté conectado y que el Monitor Serie de Arduino esté cerrado.")