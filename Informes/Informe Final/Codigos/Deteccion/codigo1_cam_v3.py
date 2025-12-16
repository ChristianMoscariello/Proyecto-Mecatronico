# ================================================================
# CÓDIGO 1: CAPTURA DE IMÁGENES Y ANÁLISIS TÉRMICO
# Versión con doble trigger: ENTER o comando ESP32
# ================================================================

import json
import time
import logging
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

    # --- Imagen térmica ---
    RESOLUCION_TERMICA = (24, 32)
    FILTRO_MEDIANA = 3

    # --- Carpetas ---
    SCRIPT_DIR = Path(__file__).resolve().parent
    CARPETA_TERMICAS = SCRIPT_DIR / 'imagenes_termicas_analizadas'
    CARPETA_VISUAL = SCRIPT_DIR / 'imagenes_visual'
    CARPETA_LOGS = SCRIPT_DIR / 'logs'

    # --- Delays ---
    DELAY_BOOT_CAMARA = 2
    DELAY_BOOT_CONEXIONES = 1


# ================================================================
# LOGGING
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
# CARPETAS
# ================================================================
def init_carpetas():
    for carpeta in [
        Config.CARPETA_TERMICAS,
        Config.CARPETA_VISUAL,
        Config.CARPETA_LOGS
    ]:
        carpeta.mkdir(exist_ok=True)
        logger.info(f"✓ Carpeta verificada: {carpeta}")

# ================================================================
# PUERTOS SERIALES
# ================================================================
def verificar_puertos_disponibles():
    try:
        import serial.tools.list_ports
        puertos = serial.tools.list_ports.comports()
        if puertos:
            logger.info("Puertos seriales disponibles:")
            for p in puertos:
                logger.info(f"  - {p.device}: {p.description}")
        else:
            logger.warning("No se encontraron puertos seriales")
    except Exception as e:
        logger.warning(f"No se pudo listar puertos: {e}")

def conectar_puerto(puerto, velocidad, timeout, nombre):
    try:
        ser = serial.Serial(puerto, velocidad, timeout=timeout)
        time.sleep(0.5)
        logger.info(f"✅ {nombre} conectado en {puerto}")
        return ser
    except serial.SerialException as e:
        logger.warning(f"⚠️ No se pudo conectar a {nombre} en {puerto}: {e}")
        return None

# ================================================================
# CÁMARA CSI
# ================================================================
def init_camara():
    try:
        from picamera2 import Picamera2
        picam2 = Picamera2()
        cfg = picam2.create_still_configuration(
            main={"size": Config.RESOLUCION_CAMARA}
        )
        picam2.configure(cfg)
        picam2.start()
        time.sleep(Config.DELAY_BOOT_CAMARA)
        logger.info("✅ Cámara CSI inicializada correctamente")
        return picam2
    except Exception as e:
        logger.error(f"❌ Error al inicializar cámara CSI: {e}")
        return None

# ================================================================
# AJUSTE INTERACTIVO DE PARÁMETROS
# ================================================================
def pedir_float(msg, val):
    x = input(f"{msg} [{val}]: ").strip()
    return float(x) if x else val

def pedir_int(msg, val):
    x = input(f"{msg} [{val}]: ").strip()
    return int(x) if x else val

def ajustar_parametros():
    print("\nParametros:")
    print(f"Persona(tmin,tmax,amin,amax) = ({Config.TEMP_PERSONA_MIN}, {Config.TEMP_PERSONA_MAX}, {Config.AREA_PERSONA_MIN}, {Config.AREA_PERSONA_MAX})")
    print(f"Fuego(tmin,amin) = ({Config.TEMP_INCENDIO_MIN}, {Config.AREA_INCENDIO_MIN})")

    r = input("Desea cambiar? (s/N): ").strip().lower()
    if r != 's':
        return

    print("\nVectores:")
    print("1) Persona")
    print("2) Fuego")
    op = input("Cuál editar? (1/2, Enter cancela): ").strip()

    if op == "1":
        Config.TEMP_PERSONA_MIN = pedir_float("TEMP_PERSONA_MIN", Config.TEMP_PERSONA_MIN)
        Config.TEMP_PERSONA_MAX = pedir_float("TEMP_PERSONA_MAX", Config.TEMP_PERSONA_MAX)
        Config.AREA_PERSONA_MIN = pedir_int("AREA_PERSONA_MIN", Config.AREA_PERSONA_MIN)
        Config.AREA_PERSONA_MAX = pedir_int("AREA_PERSONA_MAX", Config.AREA_PERSONA_MAX)

    elif op == "2":
        Config.TEMP_INCENDIO_MIN = pedir_float("TEMP_INCENDIO_MIN", Config.TEMP_INCENDIO_MIN)
        Config.AREA_INCENDIO_MIN = pedir_int("AREA_INCENDIO_MIN", Config.AREA_INCENDIO_MIN)

# ================================================================
# ANÁLISIS TÉRMICO
# ================================================================
def analizar_frame(array_temperaturas):
    matriz = array_temperaturas.reshape(Config.RESOLUCION_TERMICA)
    detecciones, bboxes = [], []

    # PERSONA
    fondo = matriz[matriz < Config.TEMP_PERSONA_MIN]
    temp_fondo = np.mean(fondo) if fondo.size else Config.TEMP_PERSONA_MIN - 5
    umbral = max(Config.TEMP_PERSONA_MIN, temp_fondo + 5)

    mask = (matriz > umbral) & (matriz <= Config.TEMP_PERSONA_MAX)
    labels_p, n = label(mask)

    for i in range(1, n + 1):
        area = np.sum(labels_p == i)
        if Config.AREA_PERSONA_MIN <= area <= Config.AREA_PERSONA_MAX:
            sy, sx = find_objects(labels_p)[i - 1]
            bboxes.append((sx.start, sy.start, sx.stop - sx.start, sy.stop - sy.start, "PERSON"))
            detecciones.append("Persona")

    # FUEGO
    labels_f, n = label(matriz > Config.TEMP_INCENDIO_MIN)
    for i in range(1, n + 1):
        area = np.sum(labels_f == i)
        if area >= Config.AREA_INCENDIO_MIN:
            sy, sx = find_objects(labels_f)[i - 1]
            bboxes.append((sx.start, sy.start, sx.stop - sx.start, sy.stop - sy.start, "FIRE"))
            detecciones.append("Foco de Incendio")

    return list(set(detecciones)), bboxes

# ================================================================
# IMAGEN TÉRMICA
# ================================================================
def generar_imagen_termica(mapa, detecciones, bboxes, ts):
    fig, ax = plt.subplots(figsize=(10, 7))
    sns.heatmap(np.fliplr(mapa), cmap="inferno", ax=ax)

    for x, y, w, h, t in bboxes:
        ax.add_patch(patches.Rectangle((x, y), w, h, fill=False,
                       edgecolor="red" if t == "FIRE" else "lime", linewidth=2))

    texto = "🚨 ALERTA: " + ", ".join(detecciones) if detecciones else "✓ Sin detecciones"
    fig.text(0.5, 0.02, texto, ha='center', fontsize=12, weight='bold')

    nombre = f"{'ALERTA_' if detecciones else ''}termico_{ts}.png"
    plt.savefig(Config.CARPETA_TERMICAS / nombre, dpi=80, bbox_inches='tight')
    plt.close(fig)

# ================================================================
# PROCESAMIENTO COMPLETO
# ================================================================
def procesar_frame_completo(datos, picam2, esp32):
    arr = np.array(datos["temperaturas"], dtype=np.float32)
    mapa = median_filter(arr.reshape(Config.RESOLUCION_TERMICA), size=Config.FILTRO_MEDIANA)
    ts = datetime.now().strftime("%y%m%d_%H%M%S_%f")[:-3]

    det, boxes = analizar_frame(mapa.flatten())
    generar_imagen_termica(mapa, det, boxes, ts)

    if picam2:
        picam2.capture_file(str(Config.CARPETA_VISUAL / f"visual_{ts}.jpg"))

    if esp32:
        msg = "PERSON" if "Persona" in det else "FIRE" if "Foco de Incendio" in det else "GO"
        esp32.write((json.dumps({"t": msg}) + "\n").encode())

# ================================================================
# SOLICITAR FRAME
# ================================================================
def solicitar_y_recibir_frame(m5, picam2, esp32):
    m5.write(b'GET_FRAME\n')
    while m5.readline().decode().strip() != "#START":
        pass
    while True:
        line = m5.readline().decode().strip()
        if line.startswith("{"):
            datos = json.loads(line)
            procesar_frame_completo(datos, picam2, esp32)
            break

# ================================================================
# HILO ESP32
# ================================================================
def escuchar_esp32(esp32, m5, picam2, flag_captura):
    while flag_captura['activo']:
        try:
            if esp32.in_waiting > 0:
                linea = esp32.readline().decode(errors="ignore").strip()

                if not linea:
                    continue  # línea vacía

                logger.debug(f"ESP32 -> {linea}")

                if not linea.startswith("{"):
                    continue  # no es JSON

                try:
                    msg = json.loads(linea)
                except json.JSONDecodeError:
                    logger.warning(f"JSON inválido desde ESP32: {linea}")
                    continue

                if msg.get("t") == "STABLE":
                    logger.info("🎯 STABLE recibido desde ESP32")
                    solicitar_y_recibir_frame(m5, picam2, esp32)

            time.sleep(0.05)

        except Exception as e:
            logger.error(f"Error en hilo ESP32: {e}")
            time.sleep(1)


# ================================================================
# MAIN
# ================================================================
def main():
    logger.info("Iniciando sistema...")
    init_carpetas()
    verificar_puertos_disponibles()

    picam2 = init_camara()
    m5 = conectar_puerto(Config.PUERTO_M5, Config.VELOCIDAD_M5, Config.TIMEOUT_M5, "M5StickC")
    esp32 = conectar_puerto(Config.PUERTO_ESP32, Config.VELOCIDAD_ESP32, 0.5, "ESP32")

    ajustar_parametros()

    flag = {"activo": True}
    if esp32:
        threading.Thread(target=escuchar_esp32, args=(esp32, m5, picam2, flag), daemon=True).start()

    logger.info("============================================================")
    logger.info("✅ SISTEMA LISTO")
    logger.info("   • Presiona ENTER para captura manual")
    logger.info("   • O espera comando STABLE desde ESP32")
    logger.info("   • Ctrl+C para salir")
    logger.info("============================================================")

    try:
        while True:
            input()
            solicitar_y_recibir_frame(m5, picam2, esp32)
    except KeyboardInterrupt:
        flag["activo"] = False

if __name__ == "__main__":
    main()
