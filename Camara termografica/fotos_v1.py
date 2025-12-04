# ================================================================
# CÓDIGO 2: CAPTURA Y ALMACENAMIENTO SIMPLE DE IMÁGENES IR Y CSI
# ================================================================
import os
import json
import time
import logging
import serial
from datetime import datetime
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import seaborn as sns

# ================================================================
# CONFIGURACIÓN GENERAL
# ================================================================
class Config:
    """Centraliza toda la configuración del sistema"""
    
    # --- Puertos Seriales ---
    PUERTO_M5 = '/dev/ttyUSB0'
    VELOCIDAD_M5 = 115200
    TIMEOUT_M5 = 3
    
    # --- Cámara CSI ---
    RESOLUCION_CAMARA = (1920, 1080)
    
    # --- Sensor Térmico ---
    RESOLUCION_TERMICA = (24, 32)  # MLX90640
    TEMP_MIN = 20
    TEMP_MAX = 60

    # --- Carpetas de salida ---
    SCRIPT_DIR = Path(__file__).resolve().parent
    CARPETA_IR = SCRIPT_DIR / 'Imagenes_IR'
    CARPETA_CSI = SCRIPT_DIR / 'Imagenes_CSI'
    CARPETA_LOGS = SCRIPT_DIR / 'logs'
    
    # --- Delays ---
    DELAY_BOOT_CAMARA = 2

# ================================================================
# CONFIGURACIÓN DE LOGGING
# ================================================================
def setup_logging():
    """Configura logging"""
    Config.CARPETA_LOGS.mkdir(exist_ok=True)
    log_file = Config.CARPETA_LOGS / f"captura_simple_{datetime.now().strftime('%y%m%d_%H%M%S')}.log"
    
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(levelname)s - %(message)s',
        handlers=[logging.FileHandler(log_file), logging.StreamHandler()]
    )
    return logging.getLogger(__name__)

logger = setup_logging()

# ================================================================
# INICIALIZACIÓN DE CARPETAS
# ================================================================
def init_carpetas():
    """Crea carpetas necesarias"""
    for carpeta in [Config.CARPETA_IR, Config.CARPETA_CSI, Config.CARPETA_LOGS]:
        carpeta.mkdir(exist_ok=True)
        logger.info(f"✓ Carpeta verificada: {carpeta}")

# ================================================================
# CONEXIÓN SERIAL AL M5
# ================================================================
def conectar_m5():
    """Conecta al M5StickC que entrega datos IR"""
    import serial.tools.list_ports
    try:
        ports = [p.device for p in serial.tools.list_ports.comports()]
        logger.info(f"Puertos disponibles: {ports}")
        
        m5 = serial.Serial(Config.PUERTO_M5, Config.VELOCIDAD_M5, timeout=Config.TIMEOUT_M5)
        time.sleep(0.5)
        logger.info(f"✅ M5StickC conectado en {Config.PUERTO_M5}")
        return m5
    except Exception as e:
        logger.error(f"❌ No se pudo conectar al M5StickC: {e}")
        return None

# ================================================================
# INICIALIZACIÓN DE CÁMARA CSI (MEJORADA)
# ================================================================
def init_camara_csi():
    """Inicializa la cámara CSI con ajustes ópticos mejorados"""
    try:
        from picamera2 import Picamera2
        from libcamera import controls

        picam2 = Picamera2()
        config = picam2.create_still_configuration(
            main={"size": Config.RESOLUCION_CAMARA},
            buffer_count=2
        )
        picam2.configure(config)
        picam2.start()
        time.sleep(Config.DELAY_BOOT_CAMARA)

        # --- Ajustes de color, exposición y balance ---
        controls_dict = {
            "AwbMode": controls.AwbModeEnum.Greyworld,  # Balance neutro
            "ExposureTime": 8000,       # µs (ajustable entre 5000–15000)
            "AnalogueGain": 1.5,        # Menos de 2.0 evita ruido
            "Brightness": 0.05,
            "Contrast": 1.2,
            "Saturation": 1.3,
            "Sharpness": 1.5
        }
        picam2.set_controls(controls_dict)

        logger.info("✅ Cámara CSI inicializada con corrección de color mejorada")
        return picam2

    except Exception as e:
        logger.error(f"❌ Error al inicializar cámara CSI: {e}")
        return None


# ================================================================
# CAPTURA VISUAL (CSI) MEJORADA
# ================================================================
def capturar_imagen_csi(picam2):
    """Captura y guarda una imagen de la cámara CSI con control de exposición adaptativo"""
    timestamp = datetime.now().strftime("%m%d-%H%M%S")
    nombre_archivo = f"CSI-{timestamp}.jpg"
    ruta = Config.CARPETA_CSI / nombre_archivo

    try:
        # Ajuste dinámico antes de captura
        picam2.autofocus_cycle()  # si el lente lo permite (no siempre en OV5647)
        picam2.set_controls({"AeEnable": True})  # Auto Exposure ON

        picam2.capture_file(str(ruta))
        logger.info(f"📸 Imagen CSI mejorada guardada: {ruta.name}")
        return ruta
    except Exception as e:
        logger.error(f"Error capturando CSI: {e}")
        return None


# ================================================================
# CAPTURA TÉRMICA (IR)
# ================================================================
def capturar_imagen_ir(m5_serial):
    """Solicita frame IR al M5 y genera mapa térmico"""
    if not m5_serial:
        logger.error("❌ No hay conexión con el M5StickC")
        return None

    try:
        # Solicita frame
        m5_serial.write(b'GET_FRAME\n')
        logger.info("📡 Solicitando frame térmico al M5StickC...")
        
        # Espera inicio
        while True:
            linea = m5_serial.readline().decode('utf-8', errors='ignore').strip()
            if linea == "#START":
                break
            if not linea:
                logger.warning("⌛ Timeout esperando #START")
                return None
        
        # Lee JSON
        json_str = ""
        while True:
            linea = m5_serial.readline().decode('utf-8', errors='ignore').strip()
            if linea == "#END":
                break
            if linea:
                json_str = linea
        
        datos = json.loads(json_str)
        temperaturas = datos.get("temperaturas")
        if not temperaturas or len(temperaturas) != 768:
            logger.error("Datos de temperatura inválidos")
            return None
        
        matriz = np.array(temperaturas).reshape(Config.RESOLUCION_TERMICA)
        timestamp = datetime.now().strftime("%m%d-%H%M%S")
        nombre_archivo = f"IR-{timestamp}.png"
        ruta = Config.CARPETA_IR / nombre_archivo
        
        # Genera mapa térmico con escala
        fig, ax = plt.subplots(figsize=(8, 6))
        sns.heatmap(
            np.fliplr(matriz),
            ax=ax,
            cmap="inferno",
            cbar_kws={'label': 'Temperatura (°C)'},
            vmin=Config.TEMP_MIN,
            vmax=Config.TEMP_MAX
        )
        ax.set_title(f"Mapa térmico - {datetime.now().strftime('%d/%m/%Y %H:%M:%S')}")
        plt.savefig(ruta, bbox_inches='tight', dpi=100)
        plt.close(fig)
        
        logger.info(f"🌡️ Imagen IR guardada: {ruta.name}")
        return ruta
    
    except Exception as e:
        logger.error(f"Error capturando IR: {e}")
        return None

# ================================================================
# BUCLE PRINCIPAL
# ================================================================
def main():
    logger.info("=" * 60)
    logger.info("INICIANDO CAPTURA SIMPLE IR + CSI")
    logger.info("Presiona ENTER para capturar | Ctrl+C para salir")
    logger.info("=" * 60)
    
    init_carpetas()
    m5_serial = conectar_m5()
    picam2 = init_camara_csi()
    
    if not m5_serial and not picam2:
        logger.error("❌ No se detectaron dispositivos. Abortando.")
        return

    try:
        while True:
            input("⌨️  Presiona ENTER para capturar... ")
            if picam2:
                capturar_imagen_csi(picam2)
            if m5_serial:
                capturar_imagen_ir(m5_serial)
    except KeyboardInterrupt:
        logger.info("🛑 Programa detenido por usuario")
    finally:
        if picam2:
            try:
                picam2.stop()
            except:
                pass
        if m5_serial and m5_serial.is_open:
            m5_serial.close()
        logger.info("✅ Sistema finalizado correctamente")

# ================================================================
if __name__ == "__main__":
    main()
