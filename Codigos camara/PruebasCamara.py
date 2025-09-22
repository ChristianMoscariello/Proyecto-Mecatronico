#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Captura fotos con Raspberry Pi Camera:
- Disparo por Enter (modo teclado) o por GPIO (sensor) con antirrebote.
- Guarda con timestamp y registra CSV (ruta, hora, brillo promedio).
- Usa Picamera2 (libcamera). No necesita entorno gráfico.

Prerrequisitos:
  sudo apt update
  sudo apt install -y python3-picamera2
  pip install opencv-python numpy RPi.GPIO

Ejecutar:
  python3 capturar_fotos.py
"""

import os, csv, time, threading
from datetime import datetime
from pathlib import Path

import numpy as np
import cv2
from picamera2 import Picamera2
try:
    import RPi.GPIO as GPIO
except ImportError:
    GPIO = None  # permite correr en PC sin GPIO

# ==== CONFIGURACIÓN ====
SAVE_DIR         = Path.home() / "fotos"
CSV_PATH         = SAVE_DIR / "capturas_log.csv"
USE_GPIO_TRIGGER = False       # True = usar GPIO, False = usar Enter
GPIO_PIN         = 17          # BCM 17 (pin físico 11)
DEBOUNCE_MS      = 250         # antirrebote (milisegundos)
MIN_INTERVAL_S   = 1.5         # tiempo mínimo entre disparos reales
RESOLUTION       = (1920, 1080)  # 1920x1080 recomendado para pruebas
AWB_MODE         = "auto"        # balance blancos: auto|incandescent|tungsten|fluorescent|daylight|cloudy
EXPOSURE_MODE    = "auto"        # exposición: auto (para comenzar)

# ==== ESTADO ====
_last_shot_ts = 0.0
_lock = threading.Lock()

def ensure_dirs_and_csv():
    SAVE_DIR.mkdir(parents=True, exist_ok=True)
    # crear CSV con encabezado si no existe
    if not CSV_PATH.exists():
        with open(CSV_PATH, "w", newline="") as f:
            cw = csv.writer(f)
            cw.writerow(["timestamp_iso", "filepath", "mean_brightness", "width", "height"])

def build_filename():
    now = datetime.now().strftime("%Y%m%d-%H%M%S")
    return SAVE_DIR / f"IMG_{now}.jpg"

def log_capture(path, img):
    mean_brightness = float(np.mean(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)))
    with open(CSV_PATH, "a", newline="") as f:
        cw = csv.writer(f)
        cw.writerow([datetime.now().isoformat(), str(path), f"{mean_brightness:.2f}", img.shape[1], img.shape[0]])
    print(f"[OK] Guardado: {path} | brillo≈{mean_brightness:.1f}")

def throttled_ok():
    global _last_shot_ts
    now = time.time()
    if (now - _last_shot_ts) >= MIN_INTERVAL_S:
        _last_shot_ts = now
        return True
    return False

def setup_camera():
    picam2 = Picamera2()
    # Configuración de preview/captura
    config = picam2.create_still_configuration(
        main={"size": RESOLUTION, "format": "RGB888"},  # RGB888 para fácil uso con OpenCV
        buffer_count=2
    )
    picam2.configure(config)

    # Controles básicos (auto para comenzar; luego podés fijar exposición/ganancia)
    controls = {}
    # Nota: los nombres exactos de controles pueden variar por versión.
    # Picamera2 maneja AWB/Exposure en "Auto" por defecto; los mantenemos.
    picam2.set_controls(controls)

    picam2.start()
    time.sleep(0.3)  # breve warmup
    return picam2

def capture_frame(picam2):
    # Obtiene frame como numpy RGB y lo convierte a BGR para OpenCV
    rgb = picam2.capture_array("main")
    bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
    return bgr

def do_capture(picam2):
    with _lock:
        if not throttled_ok():
            return
        img = capture_frame(picam2)
        path = build_filename()
        cv2.imwrite(str(path), img, [cv2.IMWRITE_JPEG_QUALITY, 92])
        log_capture(path, img)

def keyboard_loop(picam2):
    print("Modo TECLADO: presioná Enter para sacar foto. Ctrl+C para salir.")
    try:
        while True:
            input()  # espera Enter
            do_capture(picam2)
    except KeyboardInterrupt:
        print("\nSaliendo...")

def gpio_callback(channel):
    # Se dispara en flanco; aplicamos debounce por soft via throttled_ok()
    do_capture(gpio_callback.picam2)

def gpio_loop(picam2):
    if GPIO is None:
        raise RuntimeError("RPi.GPIO no disponible. Instalá en la Pi y ejecutá en hardware real.")
    print(f"Modo GPIO: escuchando en BCM{GPIO_PIN}. Ctrl+C para salir.")
    gpio_callback.picam2 = picam2
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(GPIO_PIN, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
    GPIO.add_event_detect(GPIO_PIN, GPIO.RISING, callback=gpio_callback, bouncetime=DEBOUNCE_MS)
    try:
        while True:
            time.sleep(0.5)  # loop ocioso
    except KeyboardInterrupt:
        print("\nSaliendo...")
    finally:
        GPIO.cleanup()

def main():
    ensure_dirs_and_csv()
    picam2 = setup_camera()
    print(f"Guardando en: {SAVE_DIR}")
    if USE_GPIO_TRIGGER:
        gpio_loop(picam2)
    else:
        keyboard_loop(picam2)
    picam2.stop()

if __name__ == "__main__":
    main()
