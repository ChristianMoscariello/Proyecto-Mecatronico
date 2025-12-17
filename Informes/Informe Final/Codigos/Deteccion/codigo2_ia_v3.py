#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
CÓDIGO 2: ANÁLISIS IA DE IMÁGENES RGB
- Observa carpeta de imágenes del Código 1 (imagenes_visual)
- Analiza con YOLOv8 (best_fixed.onnx)
- Envía alertas específicas al ESP32:
      PERSON_CSI
      FIRE_CSI  (para fuego)
      GO_CSI
"""

import os
import sys
import time
import json
import logging
from pathlib import Path
from datetime import datetime
from dataclasses import dataclass
import serial
import cv2
import numpy as np
import onnxruntime as ort
from watchdog.observers import Observer
from watchdog.events import FileSystemEventHandler

# ================================================================
# CONFIGURACIÓN DE LOGGING
# ================================================================
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s [%(levelname)s] %(message)s',
    datefmt='%Y-%m-%d %H:%M:%S'
)
logger = logging.getLogger(__name__)

# ================================================================
# CONFIGURACIÓN
# ================================================================
@dataclass
class Config:
    """Configuración centralizada"""
    SCRIPT_DIR: str = str(Path(__file__).resolve().parent)
    
    # === NUEVA CARPETA MONITOREADA ===
    CARPETA_MONITOREAR: str = '/home/emi/deteccion/codigo1_camaras/venv1/imagenes_visual'
    
    CARPETA_SALIDA: str = os.path.join(SCRIPT_DIR, 'imagenes_ia_analizadas')
    # Usamos el modelo corregido
    MODELO_ONNX: str = os.path.join(SCRIPT_DIR, 'best_fixed.onnx')
    CARPETA_LOGS: str = os.path.join(SCRIPT_DIR, 'logs_ia')

    # Puerto Serial ESP32
    PUERTO_ESP32: str = '/dev/serial0'
    VELOCIDAD_ESP32: int = 9600

    # Parámetros IA
    CONFIANZA_MIN: float = 0.5
    TAMAÑO_MODELO: int = 640
    CLASES: list = None

    def __post_init__(self):
        if self.CLASES is None:
            # 0=Fuego, 1=Persona
            self.CLASES = ["Fuego", "Persona"]
        self._crear_carpetas()

    def _crear_carpetas(self):
        for carpeta in [self.CARPETA_SALIDA, self.CARPETA_LOGS]:
            Path(carpeta).mkdir(exist_ok=True, parents=True)
        logger.info(f"Carpetas verificadas: {self.CARPETA_SALIDA}, {self.CARPETA_LOGS}")

# ================================================================
# GESTOR DE CONEXIÓN SERIE
# ================================================================
class GestorSerial:
    """Gestiona la conexión con ESP32"""

    def __init__(self, puerto: str, velocidad: int):
        self.puerto = puerto
        self.velocidad = velocidad
        self.conexion = None
        self.conectar()

    def conectar(self):
        try:
            # === CORRECCIÓN AQUÍ ===
            self.conexion = serial.Serial(
                self.puerto,
                self.velocidad,
                timeout=2
            )
            logger.info(f"✅ Conexión ESP32 establecida en {self.puerto}")
        except serial.SerialException as e:
            logger.warning(f"⚠️ ESP32 no disponible: {e}")
            self.conexion = None

    def enviar_alerta(self, tipo: str) -> bool:
        """Envía alerta al ESP32 en formato JSON"""
        if not self.conexion or not self.conexion.is_open:
            logger.warning("ESP32 no conectado, alerta no enviada")
            return False

        try:
            mensaje = json.dumps({"t": tipo}) + "\n"
            self.conexion.write(mensaje.encode('utf-8'))
            logger.info(f"📨 Alerta enviada a ESP32: {tipo}")
            return True

        except Exception as e:
            logger.error(f"Error enviando alerta: {e}")
            return False

    def cerrar(self):
        if self.conexion and self.conexion.is_open:
            self.conexion.close()
            logger.info("Conexión ESP32 cerrada")

# ================================================================
# ANALIZADOR IA
# ================================================================
class AnalizadorIA:
    """Analiza imágenes con YOLOv8 ONNX"""

    def __init__(self, ruta_modelo: str, config: Config):
        self.config = config
        self.session = None
        self.input_name = None
        self.output_name = None
        self._cargar_modelo(ruta_modelo)

    def _cargar_modelo(self, ruta_modelo: str):
        if not os.path.exists(ruta_modelo):
            logger.error(f"Modelo no encontrado: {ruta_modelo}")
            sys.exit(1)

        try:
            logger.info(f"Cargando modelo: {ruta_modelo}")
            self.session = ort.InferenceSession(
                ruta_modelo,
                providers=["CPUExecutionProvider"]
            )
            self.input_name = self.session.get_inputs()[0].name
            self.output_name = self.session.get_outputs()[0].name
            logger.info("✅ Modelo cargado correctamente")

        except Exception as e:
            logger.error(f"Error cargando modelo: {e}")
            sys.exit(1)

    def preprocesar_imagen(self, imagen):
        h_original, w_original = imagen.shape[:2]

        img_redimensionada = cv2.resize(
            imagen,
            (self.config.TAMAÑO_MODELO, self.config.TAMAÑO_MODELO)
        )
        img_rgb = cv2.cvtColor(img_redimensionada, cv2.COLOR_BGR2RGB)
        img_normalizada = img_rgb.astype(np.float32) / 255.0
        img_chw = np.transpose(img_normalizada, (2, 0, 1))
        img_final = np.expand_dims(img_chw, 0)

        return img_final, w_original, h_original

    def ejecutar_inferencia(self, img_pre, w_orig, h_orig):
        salida_raw = self.session.run(
            [self.output_name],
            {self.input_name: img_pre}
        )[0]

        # Convertir a formato (N,7)
        salida = salida_raw[0].T if len(salida_raw.shape) == 3 else salida_raw.T

        scale_x = w_orig / self.config.TAMAÑO_MODELO
        scale_y = h_orig / self.config.TAMAÑO_MODELO

        x_center = salida[:, 0] * scale_x
        y_center = salida[:, 1] * scale_y
        width = salida[:, 2] * scale_x
        height = salida[:, 3] * scale_y

        salida[:, 0] = x_center - width / 2
        salida[:, 1] = y_center - height / 2
        salida[:, 2] = x_center + width / 2
        salida[:, 3] = y_center + height / 2

        return salida

    # === Retorna clases detectadas ===
    def _dibujar_detecciones(self, imagen, resultados):
        hubo_detecciones = False
        clases_detectadas = []

        colores = {
            0: (0, 0, 255),      # Fuego (0) -> Rojo
            1: (0, 255, 0),      # Persona (1) -> Verde
        }

        for det in resultados:
            x1, y1, x2, y2 = det[0:4].astype(int)
            confs = det[4:]
            clase = np.argmax(confs)
            confianza = confs[clase]

            if confianza < self.config.CONFIANZA_MIN:
                continue

            hubo_detecciones = True
            clases_detectadas.append(clase)

            color = colores.get(clase, (255, 255, 255))
            cv2.rectangle(imagen, (x1, y1), (x2, y2), color, 2)

            etiqueta = f"{self.config.CLASES[clase]} ({confianza:.2f})"
            pos_y = y1 - 10 if y1 > 20 else y1 + 25

            cv2.putText(
                imagen,
                etiqueta,
                (x1, pos_y),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                color,
                2
            )

        return imagen, hubo_detecciones, clases_detectadas

    def analizar_imagen(self, ruta_imagen):
        imagen = cv2.imread(ruta_imagen)
        if imagen is None:
            logger.error(f"No se pudo leer: {ruta_imagen}")
            return None, False, []

        img_pre, w_orig, h_orig = self.preprocesar_imagen(imagen)
        resultados = self.ejecutar_inferencia(img_pre, w_orig, h_orig)

        return self._dibujar_detecciones(imagen, resultados)

# ================================================================
# EVENTOS DEL WATCHDOG
# ================================================================
class ManejadorEventos(FileSystemEventHandler):
    """Actúa cuando se detectan nuevas imágenes"""

    def __init__(self, analizador, esp32, config):
        self.analizador = analizador
        self.esp32 = esp32
        self.config = config
        self.archivos_procesados = set()

    def on_created(self, evento):
        if evento.is_directory:
            return
        if not evento.src_path.lower().endswith(('.jpg', '.jpeg', '.png')):
            return
        if evento.src_path in self.archivos_procesados:
            return

        time.sleep(0.5)
        self._procesar_imagen(evento.src_path)
        self.archivos_procesados.add(evento.src_path)

    # === Nueva lógica CSI ===
    def _procesar_imagen(self, ruta_imagen):
        try:
            nombre_archivo = Path(ruta_imagen).name
            logger.info(f"Procesando: {nombre_archivo}")

            imagen_analizada, hubo_det, clases = self.analizador.analizar_imagen(ruta_imagen)

            if imagen_analizada is None:
                logger.error(f"No se pudo procesar: {nombre_archivo}")
                return

            timestamp = datetime.now().strftime("%y%m%d_%H%M%S_%f")[:-3]
            nombre_salida = (
                f"ALERTA_ia_{timestamp}.jpg" if hubo_det else f"normal_ia_{timestamp}.jpg"
            )
            ruta_salida = os.path.join(self.config.CARPETA_SALIDA, nombre_salida)
            cv2.imwrite(ruta_salida, imagen_analizada)

            # === LÓGICA DE ENVÍO DE ALERTAS ===
            if not hubo_det:
                logger.info(f"✓ Procesado sin detecciones: {nombre_salida}")
                self.esp32.enviar_alerta("GO")
                return

            logger.warning(f"⚠️ DETECCIONES: {clases} — {nombre_salida}")

            # Fuego (Ahora es clase 0)
            if 0 in clases:
                self.esp32.enviar_alerta("FIRE")
                
            # Persona (Ahora es clase 1)
            if 1 in clases:
                self.esp32.enviar_alerta("PERSON")

        except Exception as e:
            logger.error(f"Error procesando imagen: {e}")

# ================================================================
# SISTEMA PRINCIPAL
# ================================================================
class SistemaIA:

    def __init__(self, config: Config):
        self.config = config
        self.analizador = AnalizadorIA(config.MODELO_ONNX, config)
        self.esp32 = GestorSerial(config.PUERTO_ESP32, config.VELOCIDAD_ESP32)
        self.observer = None

    def inicializar(self):
        logger.info("=" * 60)
        logger.info("INICIANDO CÓDIGO 2 - ANÁLISIS IA")
        logger.info("=" * 60)

        if not os.path.exists(self.config.CARPETA_MONITOREAR):
            logger.error(f"Carpeta no existe: {self.config.CARPETA_MONITOREAR}")
            sys.exit(1)

    def ejecutar(self):
        self.inicializar()

        handler = ManejadorEventos(self.analizador, self.esp32, self.config)
        self.observer = Observer()
        self.observer.schedule(handler, self.config.CARPETA_MONITOREAR, recursive=False)

        logger.info("\n" + "=" * 60)
        logger.info("SISTEMA IA ACTIVO — Esperando imágenes CSI…")
        logger.info("=" * 60 + "\n")

        try:
            self.observer.start()
            while True:
                time.sleep(1)

        except KeyboardInterrupt:
            logger.info("\n⚠️ Sistema detenido por el usuario")

        finally:
            self.cerrar()

    def cerrar(self):
        if self.observer:
            self.observer.stop()
            self.observer.join()
        self.esp32.cerrar()
        logger.info("Sistema IA finalizado correctamente")

# ================================================================
# MAIN
# ================================================================
def main():
    config = Config()
    sistema = SistemaIA(config)
    sistema.ejecutar()

if __name__ == "__main__":
    main()