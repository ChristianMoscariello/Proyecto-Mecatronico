#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Sistema de Detección Térmica con Análisis de Personas e Incendios
Optimizado para Raspberry Pi con M5StickC y ESP32
"""

import json
import time
from datetime import datetime
from pathlib import Path
from dataclasses import dataclass
from typing import List, Tuple, Optional
import logging
from contextlib import contextmanager

import serial
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.patheffects as PathEffects
import matplotlib.patches as patches
import seaborn as sns
from scipy.ndimage import label, median_filter, find_objects
from picamera2 import Picamera2

# ----------------------------------------------------------------
# CONFIGURACIÓN DE LOGGING
# ----------------------------------------------------------------
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s [%(levelname)s] %(message)s',
    datefmt='%Y-%m-%d %H:%M:%S'
)
logger = logging.getLogger(__name__)

# ----------------------------------------------------------------
# CONFIGURACIÓN DEL SISTEMA
# ----------------------------------------------------------------
@dataclass
class Config:
    """Configuración centralizada del sistema"""
    # Puertos Serie
    PUERTO_M5: str = '/dev/ttyUSB0'
    VELOCIDAD_M5: int = 115200
    PUERTO_ESP32: str = '/dev/serial0'
    VELOCIDAD_ESP32: int = 9600
    TIMEOUT_SERIAL: float = 2.0
    
    # Cámara
    RESOLUCION_CAMARA: Tuple[int, int] = (1920, 1080)
    
    # Detección de Personas
    TEMP_PERSONA_MIN: float = 25.0
    TEMP_PERSONA_MAX: float = 38.0
    AREA_PERSONA_MIN: int = 3
    AREA_PERSONA_MAX: int = 40
    UMBRAL_FONDO_PERSONA: float = 5.0
    
    # Detección de Incendios
    TEMP_INCENDIO_MIN: float = 50.0
    AREA_INCENDIO_MIN: int = 2
    
    # Procesamiento de Imagen
    DIMENSIONES_SENSOR: Tuple[int, int] = (24, 32)
    TAMAÑO_FILTRO_MEDIANA: int = 3
    
    # Carpetas de Salida
    CARPETA_TERMICA: str = 'imagenes_termicas_analizadas'
    CARPETA_VISUAL: str = 'imagenes_visual_deteccion'
    
    # Tiempos
    TIEMPO_INICIO_CAMARA: float = 2.0
    TIEMPO_RECONEXION: float = 5.0

# ----------------------------------------------------------------
# CLASES DE DETECCIÓN
# ----------------------------------------------------------------
@dataclass
class BoundingBox:
    """Representa un bounding box de detección"""
    x: int
    y: int
    width: int
    height: int
    
    def to_rect(self) -> patches.Rectangle:
        """Convierte a un rectángulo de matplotlib"""
        return patches.Rectangle(
            (self.x, self.y), 
            self.width, 
            self.height,
            linewidth=2, 
            edgecolor='lime', 
            facecolor='none'
        )

@dataclass
class DetectionResult:
    """Resultado de una detección"""
    tipos: List[str]
    bboxes: List[BoundingBox]
    
    def tiene_detecciones(self) -> bool:
        return len(self.tipos) > 0
    
    def tiene_persona(self) -> bool:
        return "Persona" in self.tipos
    
    def tiene_incendio(self) -> bool:
        return "Foco de Incendio" in self.tipos

# ----------------------------------------------------------------
# GESTOR DE HARDWARE
# ----------------------------------------------------------------
class HardwareManager:
    """Gestiona las conexiones de hardware del sistema"""
    
    def __init__(self, config: Config):
        self.config = config
        self.picam2: Optional[Picamera2] = None
        self.m5_serial: Optional[serial.Serial] = None
        self.esp32_serial: Optional[serial.Serial] = None
    
    def inicializar_camara(self) -> bool:
        """Inicializa la cámara CSI"""
        try:
            logger.info("Inicializando cámara CSI...")
            self.picam2 = Picamera2()
            camera_config = self.picam2.create_still_configuration(
                main={"size": self.config.RESOLUCION_CAMARA}
            )
            self.picam2.configure(camera_config)
            self.picam2.start()
            time.sleep(self.config.TIEMPO_INICIO_CAMARA)
            logger.info("✅ Cámara CSI inicializada")
            return True
        except Exception as e:
            logger.error(f"❌ Error inicializando cámara: {e}")
            self.picam2 = None
            return False
    
    def conectar_serial(self, puerto: str, velocidad: int, nombre: str) -> Optional[serial.Serial]:
        """Conecta a un dispositivo serial"""
        try:
            logger.info(f"Conectando a {nombre} en {puerto}...")
            conn = serial.Serial(puerto, velocidad, timeout=self.config.TIMEOUT_SERIAL)
            logger.info(f"✅ Conexión con {nombre} establecida")
            return conn
        except serial.SerialException as e:
            logger.error(f"❌ Error conectando a {nombre}: {e}")
            return None
    
    def inicializar_todo(self) -> bool:
        """Inicializa todo el hardware"""
        camara_ok = self.inicializar_camara()
        
        self.m5_serial = self.conectar_serial(
            self.config.PUERTO_M5,
            self.config.VELOCIDAD_M5,
            "M5StickC"
        )
        
        self.esp32_serial = self.conectar_serial(
            self.config.PUERTO_ESP32,
            self.config.VELOCIDAD_ESP32,
            "ESP32"
        )
        
        return self.m5_serial is not None and self.esp32_serial is not None
    
    def capturar_imagen(self, ruta: Path) -> bool:
        """Captura una imagen con la cámara"""
        if not self.picam2:
            return False
        try:
            self.picam2.capture_file(str(ruta))
            logger.info(f"📸 Foto capturada: {ruta.name}")
            return True
        except Exception as e:
            logger.error(f"❌ Error capturando imagen: {e}")
            return False
    
    def enviar_alerta(self, tipo: str) -> bool:
        """Envía alerta al ESP32"""
        if not self.esp32_serial:
            return False
        try:
            mensaje = f'{{"t":"{tipo}"}}\n'
            self.esp32_serial.write(mensaje.encode('utf-8'))
            logger.info(f"📨 Alerta enviada: {tipo}")
            return True
        except Exception as e:
            logger.error(f"❌ Error enviando alerta: {e}")
            return False
    
    def cleanup(self):
        """Limpia y cierra todas las conexiones"""
        if self.picam2 and self.picam2.started:
            logger.info("Deteniendo cámara...")
            self.picam2.stop()
        
        if self.m5_serial and self.m5_serial.is_open:
            self.m5_serial.close()
        
        if self.esp32_serial and self.esp32_serial.is_open:
            self.esp32_serial.close()
        
        logger.info("Hardware cerrado correctamente")

# ----------------------------------------------------------------
# DETECTOR TÉRMICO
# ----------------------------------------------------------------
class ThermalDetector:
    """Analiza imágenes térmicas y detecta personas/incendios"""
    
    def __init__(self, config: Config):
        self.config = config
    
    def analizar_frame(self, frame_array: np.ndarray) -> DetectionResult:
        """Analiza un frame térmico completo"""
        matriz = frame_array.reshape(self.config.DIMENSIONES_SENSOR)
        detecciones = []
        bboxes = []
        
        # Detección de personas
        personas_bboxes = self._detectar_personas(matriz)
        if personas_bboxes:
            detecciones.append("Persona")
            bboxes.extend(personas_bboxes)
        
        # Detección de incendios
        incendios_bboxes = self._detectar_incendios(matriz)
        if incendios_bboxes:
            detecciones.append("Foco de Incendio")
            bboxes.extend(incendios_bboxes)
        
        return DetectionResult(tipos=detecciones, bboxes=bboxes)
    
    def _detectar_personas(self, matriz: np.ndarray) -> List[BoundingBox]:
        """Detecta personas en la matriz térmica"""
        temp_fondo = np.mean(matriz[matriz < self.config.TEMP_PERSONA_MIN])
        umbral = max(
            self.config.TEMP_PERSONA_MIN,
            temp_fondo + self.config.UMBRAL_FONDO_PERSONA
        )
        
        mascara = (matriz > umbral) & (matriz <= self.config.TEMP_PERSONA_MAX)
        labels, num_features = label(mascara)
        
        if num_features == 0:
            return []
        
        bboxes = []
        slices = find_objects(labels)
        
        for i in range(1, num_features + 1):
            area = np.sum(labels == i)
            if self.config.AREA_PERSONA_MIN <= area <= self.config.AREA_PERSONA_MAX:
                bbox = self._crear_bbox(slices[i-1], matriz.shape[1])
                bboxes.append(bbox)
        
        return bboxes
    
    def _detectar_incendios(self, matriz: np.ndarray) -> List[BoundingBox]:
        """Detecta focos de incendio en la matriz térmica"""
        mascara = matriz > self.config.TEMP_INCENDIO_MIN
        labels, num_features = label(mascara)
        
        if num_features == 0:
            return []
        
        bboxes = []
        slices = find_objects(labels)
        
        for i in range(1, num_features + 1):
            area = np.sum(labels == i)
            if area >= self.config.AREA_INCENDIO_MIN:
                bbox = self._crear_bbox(slices[i-1], matriz.shape[1])
                bboxes.append(bbox)
        
        return bboxes
    
    @staticmethod
    def _crear_bbox(slice_obj: Tuple, ancho_matriz: int) -> BoundingBox:
        """Crea un BoundingBox desde un objeto slice"""
        slice_y, slice_x = slice_obj
        width = slice_x.stop - slice_x.start
        height = slice_y.stop - slice_y.start
        x_flipped = ancho_matriz - slice_x.stop
        return BoundingBox(x_flipped, slice_y.start, width, height)

# ----------------------------------------------------------------
# GENERADOR DE VISUALIZACIONES
# ----------------------------------------------------------------
class VisualizationGenerator:
    """Genera las visualizaciones térmicas"""
    
    def __init__(self, config: Config):
        self.config = config
        self.carpeta_termica = Path(config.CARPETA_TERMICA)
        self.carpeta_visual = Path(config.CARPETA_VISUAL)
        self._crear_carpetas()
    
    def _crear_carpetas(self):
        """Crea las carpetas de salida si no existen"""
        for carpeta in [self.carpeta_termica, self.carpeta_visual]:
            carpeta.mkdir(exist_ok=True)
            logger.info(f"Carpeta preparada: {carpeta}")
    
    def generar_mapa_termico(
        self,
        matriz: np.ndarray,
        resultado: DetectionResult,
        timestamp: str
    ) -> Path:
        """Genera y guarda el mapa térmico"""
        # Preparar nombre de archivo
        prefijo = "ALERTA" if resultado.tiene_detecciones() else "normal"
        nombre = f"{prefijo}_mapa_{timestamp}.png"
        ruta = self.carpeta_termica / nombre
        
        # Crear visualización
        matriz_visual = np.fliplr(matriz)
        fig, ax = plt.subplots(figsize=(8, 6))
        
        sns.heatmap(
            matriz_visual,
            ax=ax,
            cmap="inferno",
            cbar_kws={'label': 'Temperatura (°C)'}
        )
        
        # Configurar título
        tiempo = datetime.now().strftime('%d/%m/%Y %H:%M:%S')
        ax.set_title(f"Mapa de Calor - {tiempo}")
        ax.set_xticks([])
        ax.set_yticks([])
        
        # Dibujar bounding boxes
        for bbox in resultado.bboxes:
            ax.add_patch(bbox.to_rect())
        
        # Agregar texto de detección
        self._agregar_texto_deteccion(fig, resultado)
        
        # Guardar
        plt.savefig(ruta, bbox_inches='tight', dpi=100)
        plt.close(fig)
        
        return ruta
    
    @staticmethod
    def _agregar_texto_deteccion(fig, resultado: DetectionResult):
        """Agrega el texto de detección al gráfico"""
        if resultado.tiene_detecciones():
            texto = "¡ALERTA!: " + ", ".join(resultado.tipos)
            color = "red"
        else:
            texto = "Sin Detecciones"
            color = "green"
        
        txt = fig.text(
            0.5, 0.05, texto,
            ha='center', va='bottom',
            fontsize=14, color=color, weight='bold'
        )
        txt.set_path_effects([
            PathEffects.withStroke(linewidth=3, foreground='black')
        ])
    
    def obtener_ruta_visual(self, timestamp: str) -> Path:
        """Obtiene la ruta para guardar una imagen visual"""
        return self.carpeta_visual / f"visual_{timestamp}.jpg"

# ----------------------------------------------------------------
# SISTEMA PRINCIPAL
# ----------------------------------------------------------------
class ThermalMonitoringSystem:
    """Sistema principal de monitoreo térmico"""
    
    def __init__(self, config: Config):
        self.config = config
        self.hardware = HardwareManager(config)
        self.detector = ThermalDetector(config)
        self.visualizer = VisualizationGenerator(config)
    
    def inicializar(self) -> bool:
        """Inicializa el sistema completo"""
        logger.info("=" * 60)
        logger.info("INICIANDO SISTEMA DE DETECCIÓN TÉRMICA")
        logger.info("=" * 60)
        return self.hardware.inicializar_todo()
    
    def procesar_frame_termico(self, datos: dict):
        """Procesa un frame térmico recibido"""
        temperaturas = datos.get('temperaturas')
        
        if not temperaturas or len(temperaturas) != 768:
            return
        
        # Preparar datos
        array_temp = np.array(temperaturas, dtype=np.float32)
        matriz_filtrada = median_filter(
            array_temp.reshape(self.config.DIMENSIONES_SENSOR),
            size=self.config.TAMAÑO_FILTRO_MEDIANA
        )
        
        # Analizar
        resultado = self.detector.analizar_frame(matriz_filtrada.flatten())
        
        # Generar timestamp
        timestamp = datetime.now().strftime("%y%m%d_%H%M%S")
        
        # Guardar mapa térmico
        ruta_termica = self.visualizer.generar_mapa_termico(
            matriz_filtrada,
            resultado,
            timestamp
        )
        
        # Procesar alertas
        if resultado.tiene_detecciones():
            self._procesar_alertas(resultado, timestamp)
        
        # Log del resultado
        estado = ", ".join(resultado.tipos) if resultado.tipos else "Normal"
        logger.info(f"Frame procesado: {ruta_termica.name} | Estado: {estado}")
    
    def _procesar_alertas(self, resultado: DetectionResult, timestamp: str):
        """Procesa las alertas detectadas"""
        if resultado.tiene_persona():
            self.hardware.enviar_alerta("PERSON")
        
        if resultado.tiene_incendio():
            self.hardware.enviar_alerta("FIRE")
        
        # Capturar imagen visual
        ruta_visual = self.visualizer.obtener_ruta_visual(timestamp)
        self.hardware.capturar_imagen(ruta_visual)
    
    def ejecutar(self):
        """Ejecuta el bucle principal del sistema"""
        if not self.inicializar():
            logger.error("❌ Error en la inicialización. Abortando.")
            return
        
        logger.info("\n" + "=" * 60)
        logger.info("SISTEMA ACTIVO - Esperando datos térmicos")
        logger.info("=" * 60 + "\n")
        
        try:
            while True:
                self._procesar_datos_serial()
                
        except KeyboardInterrupt:
            logger.info("\n⚠️ Detenido por el usuario")
        except Exception as e:
            logger.exception(f"❌ Error crítico: {e}")
        finally:
            self.hardware.cleanup()
            logger.info("Sistema finalizado")
    
    def _procesar_datos_serial(self):
        """Procesa datos del puerto serial"""
        if not self.hardware.m5_serial:
            return
        
        linea = self.hardware.m5_serial.readline()
        if not linea:
            return
        
        try:
            texto = linea.decode('utf-8', errors='ignore').strip()
            
            # Ignorar mensajes no-JSON
            if not texto.startswith('{'):
                if texto:
                    logger.debug(f"M5: {texto}")
                return
            
            # Parsear y procesar JSON
            datos = json.loads(texto)
            self.procesar_frame_termico(datos)
            
        except json.JSONDecodeError:
            pass  # Ignorar silenciosamente JSONs malformados
        except Exception as e:
            logger.error(f"Error procesando datos: {e}")

# ----------------------------------------------------------------
# PUNTO DE ENTRADA
# ----------------------------------------------------------------
def main():
    """Función principal"""
    config = Config()
    sistema = ThermalMonitoringSystem(config)
    sistema.ejecutar()

if __name__ == "__main__":
    main()