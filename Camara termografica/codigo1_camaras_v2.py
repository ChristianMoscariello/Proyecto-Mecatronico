# ================================================================
# CÓDIGO 1: CAPTURA DE IMÁGENES Y ANÁLISIS TÉRMICO
# Versión con doble trigger: ENTER o comando ESP32
# ================================================================
import json
import time
import logging
import os
import serial
import threading
from datetime import datetime
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.patheffects as PathEffects
import matplotlib.patches as patches
import seaborn as sns
from scipy.ndimage import label, median_filter, find_objects

# ================================================================
# CONFIGURACIÓN GENERAL
# ================================================================
class Config:
    """Centraliza toda la configuración del sistema"""
    
    # --- Puertos Seriales ---
    PUERTO_M5 = '/dev/ttyUSB0'
    VELOCIDAD_M5 = 115200
    TIMEOUT_M5 = 3
    
    PUERTO_ESP32 = '/dev/serial0'
    VELOCIDAD_ESP32 = 9600
    
    # --- Cámara CSI ---
    RESOLUCION_CAMARA = (1920, 1080)
    
    # --- Parámetros de Detección Térmica ---
    TEMP_PERSONA_MIN = 25.0
    TEMP_PERSONA_MAX = 38.0
    AREA_PERSONA_MIN = 3
    AREA_PERSONA_MAX = 40
    
    TEMP_INCENDIO_MIN = 50.0
    AREA_INCENDIO_MIN = 2
    
    # --- Parámetros de Imagen Térmica ---
    RESOLUCION_TERMICA = (24, 32)
    FILTRO_MEDIANA = 3
    
    # --- Carpetas de Salida ---
    SCRIPT_DIR = Path(__file__).resolve().parent
    CARPETA_TERMICAS = SCRIPT_DIR / 'imagenes_termicas_analizadas'
    CARPETA_VISUAL = SCRIPT_DIR / 'imagenes_visual'
    CARPETA_LOGS = SCRIPT_DIR / 'logs'
    
    # --- Timeouts y Delays ---
    DELAY_BOOT_CAMARA = 2
    DELAY_BOOT_CONEXIONES = 1

# ================================================================
# CONFIGURACIÓN DE LOGGING
# ================================================================
def setup_logging():
    Config.CARPETA_LOGS.mkdir(exist_ok=True)
    
    log_file = Config.CARPETA_LOGS / f"codigo1_{datetime.now().strftime('%y%m%d_%H%M%S')}.log"
    
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(levelname)s - %(message)s',
        handlers=[
            logging.FileHandler(log_file),
            logging.StreamHandler()
        ]
    )
    return logging.getLogger(__name__)

logger = setup_logging()

# ================================================================
# INICIALIZACIÓN DE CARPETAS
# ================================================================
def init_carpetas():
    carpetas = [
        Config.CARPETA_TERMICAS,
        Config.CARPETA_VISUAL,
        Config.CARPETA_LOGS
    ]
    
    for carpeta in carpetas:
        carpeta.mkdir(exist_ok=True)
        logger.info(f"✓ Carpeta verificada: {carpeta}")

# ================================================================
# MANEJO DE CONEXIONES SERIALES
# ================================================================
def conectar_puerto(puerto, velocidad, timeout, nombre_dispositivo):
    try:
        conexion = serial.Serial(puerto, velocidad, timeout=timeout)
        time.sleep(0.5)
        logger.info(f"✅ {nombre_dispositivo} conectado en {puerto}")
        return conexion
    except serial.SerialException as e:
        logger.warning(f"⚠️ No se pudo conectar a {nombre_dispositivo} en {puerto}: {e}")
        return None

def verificar_puertos_disponibles():
    try:
        import serial.tools.list_ports
        puertos = serial.tools.list_ports.comports()
        if puertos:
            logger.info("Puertos seriales disponibles:")
            for puerto in puertos:
                logger.info(f"  - {puerto.device}: {puerto.description}")
        else:
            logger.warning("No se encontraron puertos seriales")
    except Exception as e:
        logger.warning(f"No se pudo listar puertos: {e}")

# ================================================================
# INICIALIZACIÓN CÁMARA CSI
# ================================================================
def init_camara():
    try:
        from picamera2 import Picamera2
        picam2 = Picamera2()
        camera_config = picam2.create_still_configuration(
            main={"size": Config.RESOLUCION_CAMARA}
        )
        picam2.configure(camera_config)
        picam2.start()
        time.sleep(Config.DELAY_BOOT_CAMARA)
        logger.info("✅ Cámara CSI inicializada correctamente")
        return picam2
    except Exception as e:
        logger.error(f"❌ Error al inicializar cámara CSI: {e}")
        return None

# ================================================================
# ANÁLISIS DE IMAGEN TÉRMICA
# ================================================================
def analizar_frame(array_temperaturas):

    try:
        matriz_termica = array_temperaturas.reshape(Config.RESOLUCION_TERMICA)
        detecciones = []
        bboxes = []
        
        # === DETECCIÓN DE PERSONAS ===
        pixeles_fondo = matriz_termica[matriz_termica < Config.TEMP_PERSONA_MIN]
        temp_fondo = np.mean(pixeles_fondo) if pixeles_fondo.size > 0 else Config.TEMP_PERSONA_MIN - 5
        umbral_persona = max(Config.TEMP_PERSONA_MIN, temp_fondo + 5)
        
        mascara_persona = (matriz_termica > umbral_persona) & (matriz_termica <= Config.TEMP_PERSONA_MAX)
        labels_persona, num_features = label(mascara_persona)
        
        if num_features > 0:
            slice_objects = find_objects(labels_persona)
            for i in range(1, num_features + 1):
                area = np.sum(labels_persona == i)
                if Config.AREA_PERSONA_MIN <= area <= Config.AREA_PERSONA_MAX:
                    detecciones.append("Persona")
                    slice_y, slice_x = slice_objects[i - 1]
                    x, y = slice_x.start, slice_y.start
                    w, h = slice_x.stop - slice_x.start, slice_y.stop - slice_y.start
                    bboxes.append((x, y, w, h, "PERSON"))
        
        # === DETECCIÓN DE INCENDIOS ===
        mascara_incendio = matriz_termica > Config.TEMP_INCENDIO_MIN
        labels_incendio, num_features = label(mascara_incendio)
        
        if num_features > 0:
            slice_objects = find_objects(labels_incendio)
            for i in range(1, num_features + 1):
                area = np.sum(labels_incendio == i)
                if area >= Config.AREA_INCENDIO_MIN:
                    detecciones.append("Foco de Incendio")
                    slice_y, slice_x = slice_objects[i - 1]
                    x, y = slice_x.start, slice_y.start
                    w, h = slice_x.stop - slice_x.start, slice_y.stop - slice_y.start
                    bboxes.append((x, y, w, h, "FIRE"))
        
        return list(set(detecciones)), bboxes
        
    except Exception as e:
        logger.error(f"Error en análisis de frame: {e}")
        return [], []

# ================================================================
# GENERACIÓN DE IMÁGENES TÉRMICAS
# ================================================================
def generar_imagen_termica(mapa_filtrado, detecciones, bboxes, timestamp):
    try:
        fig, ax = plt.subplots(figsize=(10, 7))
        
        mapa_visual = np.fliplr(mapa_filtrado)
        sns.heatmap(
            mapa_visual, ax=ax, cmap="inferno",
            cbar_kws={'label': 'Temperatura (°C)'}
        )
        
        now = datetime.now()
        ax.set_title(f"Mapa Térmico - {now.strftime('%d/%m/%Y %H:%M:%S')}", fontsize=14, weight='bold')

        colores_bbox = {"PERSON": "lime", "FIRE": "red"}
        for x, y, w, h, tipo in bboxes:
            color = colores_bbox.get(tipo, "white")
            rect = patches.Rectangle((x, y), w, h, linewidth=2, edgecolor=color, facecolor='none')
            ax.add_patch(rect)
        
        if detecciones:
            texto = "🚨 ALERTA: " + ", ".join(detecciones)
            color_texto = "red"
            nombre_archivo = f"ALERTA_termico_{timestamp}.png"
        else:
            texto = "✓ Sin detecciones"
            color_texto = "green"
            nombre_archivo = f"termico_{timestamp}.png"
        
        txt = fig.text(0.5, 0.02, texto, ha='center', va='bottom', 
                      fontsize=12, color=color_texto, weight='bold')
        txt.set_path_effects([PathEffects.withStroke(linewidth=3, foreground='black')])
        
        ruta_salida = Config.CARPETA_TERMICAS / nombre_archivo
        plt.savefig(ruta_salida, bbox_inches='tight', dpi=80)
        plt.close(fig)
        
        logger.info(f"📊 Imagen térmica guardada: {nombre_archivo}")
        return nombre_archivo
        
    except Exception as e:
        logger.error(f"Error generando imagen térmica: {e}")
        return None

# ================================================================
# CAPTURA DE CÁMARA VISUAL (SIEMPRE MISMA CARPETA)
# ================================================================
def capturar_visual(picam2, timestamp, hay_deteccion=True):
    if not picam2:
        logger.warning("Cámara CSI no disponible")
        return None
    
    try:
        carpeta = Config.CARPETA_VISUAL   # SIEMPRE MISMA CARPETA
        nombre_archivo = f"visual_{timestamp}.jpg"
        ruta_salida = carpeta / nombre_archivo
        
        picam2.capture_file(str(ruta_salida))
        logger.info(f"📸 Imagen visual guardada: {nombre_archivo}")
        return nombre_archivo
        
    except Exception as e:
        logger.error(f"Error capturando imagen visual: {e}")
        return None

# ================================================================
# ENVÍO DE ALERTAS A ESP32
# ================================================================
def enviar_alerta_esp32(serial_conn, tipo_alerta):
    if not serial_conn:
        logger.warning("ESP32 no conectado, alerta no enviada")
        return False
    
    try:
        mensaje = json.dumps({"t": tipo_alerta}) + "\n"
        serial_conn.write(mensaje.encode('utf-8'))
        logger.info(f"📨 Alerta enviada a ESP32: {tipo_alerta}")
        return True
    except Exception as e:
        logger.error(f"Error enviando alerta a ESP32: {e}")
        return False

# ================================================================
# PROCESAMIENTO DE FRAME COMPLETO
# ================================================================
def procesar_frame_completo(datos_m5, picam2, esp32_serial):
    try:
        temperaturas = datos_m5.get('temperaturas')
        if not temperaturas or len(temperaturas) != 768:
            logger.error("Datos de temperatura inválidos o incompletos")
            return False
        
        timestamp = datetime.now().strftime("%y%m%d_%H%M%S_%f")[:-3]
        
        array_temp = np.array(temperaturas, dtype=np.float32)
        mapa_filtrado = median_filter(array_temp.reshape(Config.RESOLUCION_TERMICA), 
                                     size=Config.FILTRO_MEDIANA)
        
        detecciones, bboxes = analizar_frame(mapa_filtrado.flatten())
        
        generar_imagen_termica(mapa_filtrado, detecciones, bboxes, timestamp)
        
        # CSI SIEMPRE SE CAPTURA
        capturar_visual(picam2, timestamp, hay_deteccion=True)

        # === ALERTAS IR ===
        if detecciones:
            logger.warning(f"🚨 ALERTA DETECTADA: {detecciones}")
            
            if "Persona" in detecciones:
                enviar_alerta_esp32(esp32_serial, "PERSON")

            if "Foco de Incendio" in detecciones:
                enviar_alerta_esp32(esp32_serial, "FIRE")

        else:
            logger.info("✓ Frame procesado sin detecciones (IR)")
            enviar_alerta_esp32(esp32_serial, "GO")
            logger.info("➡️ Señal GO enviada al dron")
        
        return True
        
    except Exception as e:
        logger.error(f"Error procesando frame completo: {e}")
        return False

# ================================================================
# SOLICITUD Y RECEPCIÓN DE FRAME del M5
# ================================================================
def solicitar_y_recibir_frame(m5_serial, picam2, esp32_serial):
    try:
        logger.info("Solicitando frame al M5StickC...")
        m5_serial.write(b'GET_FRAME\n')
    except Exception as e:
        logger.error(f"Error enviando solicitud: {e}")
        return False
    
    try:
        while True:
            linea = m5_serial.readline().decode('utf-8', errors='ignore').strip()
            if linea == "#START":
                break
            if not linea:
                logger.warning("⌛️ Timeout: M5StickC no respondió")
                return False
        
        json_str = ""
        while True:
            linea = m5_serial.readline().decode('utf-8', errors='ignore').strip()
            if linea == "#END":
                break
            if linea:
                json_str = linea
        
        if not json_str.startswith('{'):
            logger.error("JSON no recibido correctamente")
            return False
        
        datos = json.loads(json_str)
        return procesar_frame_completo(datos, picam2, esp32_serial)
        
    except json.JSONDecodeError as e:
        logger.error(f"JSON inválido: {e}")
        return False
    except Exception as e:
        logger.error(f"Error procesando frame: {e}")
        return False

# ================================================================
# HILO QUE ESCUCHA ESP32
# ================================================================
def escuchar_esp32(esp32_serial, m5_serial, picam2, flag_captura):
    logger.info("🎧 Iniciado hilo de escucha ESP32")
    
    while flag_captura['activo']:
        if not esp32_serial or not esp32_serial.is_open:
            time.sleep(1)
            continue
        
        try:
            if esp32_serial.in_waiting > 0:
                linea = esp32_serial.readline().decode('utf-8', errors='ignore').strip()
                
                if linea:
                    logger.info(f"📩 Recibido desde ESP32: {linea}")
                    
                    try:
                        mensaje = json.loads(linea)
                        
                        if mensaje.get('t') == 'STABLE':
                            logger.info("🎯 Comando STABLE recibido - Iniciando captura automática")
                            solicitar_y_recibir_frame(m5_serial, picam2, esp32_serial)
                    
                    except json.JSONDecodeError:
                        pass
            
            time.sleep(0.1)
            
        except Exception as e:
            logger.error(f"Error en hilo ESP32: {e}")
            time.sleep(1)
    
    logger.info("🎧 Hilo de escucha ESP32 finalizado")

# ================================================================
# BUCLE PRINCIPAL
# ================================================================
def main():
    logger.info("=" * 60)
    logger.info("INICIANDO CÓDIGO 1 - CAPTURA Y ANÁLISIS TÉRMICO")
    logger.info("Modo: ENTER o comando ESP32 (STABLE)")
    logger.info("=" * 60)
    
    init_carpetas()
    verificar_puertos_disponibles()
    
    picam2 = init_camara()
    m5_serial = conectar_puerto(Config.PUERTO_M5, Config.VELOCIDAD_M5, 
                                Config.TIMEOUT_M5, "M5StickC")
    esp32_serial = conectar_puerto(Config.PUERTO_ESP32, Config.VELOCIDAD_ESP32, 
                                   0.5, "ESP32")
    
    if not m5_serial:
        logger.error("❌ No se pudo conectar al M5StickC. Abortando.")
        return
    
    time.sleep(Config.DELAY_BOOT_CONEXIONES)
    
    flag_captura = {'activo': True}
    
    if esp32_serial:
        hilo_esp32 = threading.Thread(
            target=escuchar_esp32, 
            args=(esp32_serial, m5_serial, picam2, flag_captura),
            daemon=True
        )
        hilo_esp32.start()
        logger.info("✅ Hilo de escucha ESP32 iniciado")
    else:
        logger.warning("⚠️ ESP32 no conectado - Solo funcionará modo ENTER")
    
    logger.info("\n" + "=" * 60)
    logger.info("✅ SISTEMA LISTO")
    logger.info("   • Presiona ENTER para captura manual")
    logger.info("   • O espera comando STABLE desde ESP32")
    logger.info("   • Ctrl+C para salir")
    logger.info("=" * 60 + "\n")
    
    try:
        while True:
            input("⌨️  Presiona ENTER para capturar frame... ")
            solicitar_y_recibir_frame(m5_serial, picam2, esp32_serial)
    
    except KeyboardInterrupt:
        logger.info("\n🛑 Programa detenido por usuario")
    
    finally:
        flag_captura['activo'] = False
        
        if picam2:
            try:
                picam2.stop()
                logger.info("Cámara CSI detenida")
            except:
                pass
        
        if m5_serial and m5_serial.is_open:
            m5_serial.close()
            logger.info("Conexión M5 cerrada")
        
        if esp32_serial and esp32_serial.is_open:
            esp32_serial.close()
            logger.info("Conexión ESP32 cerrada")
        
        logger.info("Sistema finalizado correctamente")

if __name__ == "__main__":
    main()
