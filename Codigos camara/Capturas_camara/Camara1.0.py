# ====== IMPORTS ======
# Picamera2: control de cámara en Raspberry Pi OS moderno
from picamera2 import Picamera2
# numpy solo para manejar arrays si fuera necesario (Picamera2 lo usa internamente)
import numpy as np
# Manejo de tiempos y timestamps
from datetime import datetime, timezone
# Sistema de archivos y paths
from pathlib import Path
# Pausa y espera activa
import time
# Para componer cadenas de forma segura
import re

# MAVLink (pymavlink) para leer GPS desde el Pixhawk
from pymavlink import mavutil


# ====== CONFIGURACIÓN GENERAL ======
# Carpeta donde se guardarán las fotos (se crea si no existe)
BASE_DIR = Path.cwd() / "fotos"

# Puerto serie del Pixhawk (ajustar si difiere)
# Comunes: '/dev/ttyACM0' (USB), '/dev/ttyAMA0' (UART), '/dev/ttyUSB0' (adaptador)
SERIAL_PORT = "/dev/ttyACM0"
BAUD = 57600

# Tiempo máximo (segundos) para esperar un mensaje de posición válido al capturar
GPS_TIMEOUT_S = 8.0

# Resolución de captura (ajustable según la PiCam y tus necesidades)
CAPTURE_SIZE = (1920, 1080)   # 1080p

# Compresión JPEG (1-100). 90 es buena calidad con tamaño moderado.
JPEG_QUALITY = 90


# ====== FUNCIONES DE APOYO ======
def conectar_pixhawk(port: str, baud: int):
    """
    Abre un enlace MAVLink hacia el Pixhawk.
    Retorna el objeto 'mavutil.mavlink_connection' ya sincronizado con heartbeat.
    """
    print(f"[MAVLink] Conectando a {port} @ {baud} baudios…")
    # 'device' para puerto serie; también puede ser 'udp:127.0.0.1:14550' si fuese por UDP
    master = mavutil.mavlink_connection(device=port, baud=baud)
    # Esperar al primer heartbeat para saber que tenemos comunicación
    master.wait_heartbeat(timeout=5)
    print(f"[MAVLink] Conectado. System {master.target_system}, Component {master.target_component}")
    return master


def leer_gps(master, timeout_s: float = GPS_TIMEOUT_S):
    """
    Intenta leer una solución de posición desde el Pixhawk.
    Prioriza GLOBAL_POSITION_INT por ser 'fusionada' por EKF (lat/lon/alt AMSL).
    Si no llega, intenta GPS_RAW_INT (datos directos del receptor GNSS).

    Retorna:
        dict con claves: {'lat', 'lon', 'alt', 'alt_type', 'fix_ok'}
        - lat, lon en grados decimales (float)
        - alt en metros (float)
        - alt_type: 'amsl' si viene de GLOBAL_POSITION_INT (milímetros sobre MSL),
                    'ellipsoid' si viene de GPS_RAW_INT (milímetros)
        - fix_ok: True si hay fix razonable, False si no
    """
    t0 = time.time()
    # 1) Intentar GLOBAL_POSITION_INT (lat/lon escalados en 1e7, alt en mm AMSL)
    while time.time() - t0 < timeout_s:
        msg = master.recv_match(type="GLOBAL_POSITION_INT", blocking=False)
        if msg is not None:
            lat = msg.lat / 1e7
            lon = msg.lon / 1e7
            alt_m = msg.alt / 1000.0  # mm -> m (AMSL: sobre nivel medio del mar)
            # Validación básica: lat/lon dentro de rangos
            if -90 <= lat <= 90 and -180 <= lon <= 180:
                return {'lat': lat, 'lon': lon, 'alt': alt_m, 'alt_type': 'amsl', 'fix_ok': True}
        time.sleep(0.05)

    # 2) Si no llegó GLOBAL_POSITION_INT, intentar GPS_RAW_INT (1e7 y mm sobre elipsoide)
    t1 = time.time()
    while time.time() - t1 < timeout_s:
        msg = master.recv_match(type="GPS_RAW_INT", blocking=False)
        if msg is not None:
            # fix_type: 0-1 no fix, 2=2D, 3=3D, 4=DGPS, 5=RTK Float, 6=RTK Fixed, etc.
            fix_ok = getattr(msg, "fix_type", 0) >= 3
            lat = msg.lat / 1e7
            lon = msg.lon / 1e7
            alt_m = msg.alt / 1000.0  # mm -> m (sobre elipsoide WGS84)
            # Validación básica
            if -90 <= lat <= 90 and -180 <= lon <= 180 and fix_ok:
                return {'lat': lat, 'lon': lon, 'alt': alt_m, 'alt_type': 'ellipsoid', 'fix_ok': True}
        time.sleep(0.05)

    # 3) Nada confiable
    return {'lat': None, 'lon': None, 'alt': None, 'alt_type': None, 'fix_ok': False}


def fmt_coord(value, decimals=6):
    """Formatea lat/lon con signo y número fijo de decimales: +DD.dddddd / -DDD.dddddd"""
    if value is None:
        return "NA"
    return f"{value:+0.{decimals}f}"


def safe_filename(s: str) -> str:
    """Limpia una cadena para que sea apta como nombre de archivo en Linux."""
    import re as _re
    s = _re.sub(r"[^A-Za-z0-9._+-]", "_", s)
    return s[:255]  # longitud segura


def nombre_archivo(lat, lon, alt, fix_ok, ts_utc: datetime) -> str:
    """
    Construye el nombre del archivo con timestamp UTC y coordenadas si hay fix.
    Ejemplo: FOTO_2025-09-10-21h15m30s_lat-34.612345_lon-58.381234_alt25.4m.jpg
    o si no hay fix: FOTO_2025-09-10-21h15m30s_NOFIX.jpg
    """
    ts = ts_utc.strftime("%Y-%m-%d-%Hh%Mm%Ss")
    if fix_ok:
        base = f"FOTO_{ts}_lat{fmt_coord(lat)}_lon{fmt_coord(lon)}"
        if alt is not None:
            base += f"_alt{alt:.1f}m"
    else:
        base = f"FOTO_{ts}_NOFIX"
    return safe_filename(base + ".jpg")


def preparar_camara():
    """Inicializa Picamera2 con una configuración sencilla para foto fija (still)."""
    pico = Picamera2()
    config = pico.create_still_configuration(main={"size": CAPTURE_SIZE})
    pico.configure(config)
    pico.start()
    # pequeño warm-up para auto-exposición / balance de blancos
    time.sleep(0.8)
    return pico


# ====== PROGRAMA PRINCIPAL ======
def main():
    print("=== Captura PiCam2 con GPS de Pixhawk ===")
    print(f"Carpeta destino: {BASE_DIR}")
    print(f"Puerto Pixhawk: {SERIAL_PORT} @ {BAUD}")
    BASE_DIR.mkdir(parents=True, exist_ok=True)

    # Conectar a Pixhawk por MAVLink
    try:
        master = conectar_pixhawk(SERIAL_PORT, BAUD)
    except Exception as e:
        print(f"[ERROR] No pude conectar al Pixhawk: {e}")
        print("Podés seguir capturando, pero sin coordenadas (saldrá NOFIX).")
        master = None

    # Preparar cámara
    try:
        cam = preparar_camara()
    except Exception as e:
        print(f"[ERROR] No pude inicializar la cámara: {e}")
        print("Revisá que la cámara esté habilitada y Picamera2 instalado.")
        return

    print("Listo. Presioná Enter para capturar foto. Ctrl+C para salir.")
    while True:
        try:
            input("Presioná Enter para capturar… ")
        except (KeyboardInterrupt, EOFError):
            print("\nSaliendo…")
            break

        # Timestamp en UTC para evitar ambigüedades
        now_utc = datetime.now(timezone.utc)

        # Intentar leer posición del Pixhawk si hay conexión
        if master is not None:
            pos = leer_gps(master, timeout_s=GPS_TIMEOUT_S)
        else:
            pos = {'lat': None, 'lon': None, 'alt': None, 'alt_type': None, 'fix_ok': False}

        # Componer nombre de archivo
        fname = nombre_archivo(pos['lat'], pos['lon'], pos['alt'], pos['fix_ok'], now_utc)
        fpath = BASE_DIR / fname

        # Capturar y guardar
        try:
            cam.capture_file(str(fpath), format="jpeg", quality=JPEG_QUALITY)
            if pos['fix_ok']:
                print(f"[OK] Foto guardada: {fpath.name}  ({pos['lat']:.6f}, {pos['lon']:.6f}, alt {pos['alt']:.1f} m)")
            else:
                print(f"[OK] Foto guardada: {fpath.name}  (SIN FIX GPS)")
        except Exception as e:
            print(f"[ERROR] No pude guardar la foto: {e}")

    # Apagar cámara al salir
    try:
        cam.stop()
    except Exception:
        pass


if __name__ == "__main__":
    main()
