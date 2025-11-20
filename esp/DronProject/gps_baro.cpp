#include "gps_baro.h"

// ========================================================
// DEBUG SELECTIVO
// ========================================================
#define GPS_BARO_DEBUG 0
// 0 = nada
// 1 = prints de eventos
// 2 = prints verbose

#define DBG(...) if (GPS_BARO_DEBUG >= 1) { Serial.printf(__VA_ARGS__); }

// ========================================================
// Variables globales (antes estaban en tu .ino)
// ========================================================
double lat_f = 0.0, lon_f = 0.0, alt_f = 0.0;

float alt_baro_f = 0.0f;
float climb_mps  = 0.0f;

static float alt_zero = 0.0f;
static bool  alt_zero_set = false;

// ========================================================
// Parámetros del filtro barométrico
// ========================================================
static float QNH_hPa = 1013.25;
static const float ALT_EMA_ALPHA = 0.25f;
static const float ALT_MAX_STEP  = 3.0f;
static unsigned long lastAltMs   = 0;

// ========================================================
// GPS filters internals
// ========================================================
static double lat_window[4] = {0};
static double lon_window[4] = {0};
static double alt_window[4] = {0};

static int win_idx = 0;
static bool have_filter = false;

static int gpsWarmup = 0;
const int GPS_WARMUP_COUNT = 10;
const double MAX_JUMP_DEG = 0.00030;

// ========================================================
//  INIT BARO
// ========================================================
void initBaro() {
    if (bmp_ok) {
        bmp.setSampling(
            Adafruit_BMP280::MODE_NORMAL,
            Adafruit_BMP280::SAMPLING_X2,
            Adafruit_BMP280::SAMPLING_X16,
            Adafruit_BMP280::FILTER_X16,
            Adafruit_BMP280::STANDBY_MS_63
        );
        DBG("BMP280 listo\n");
    } else {
        Serial.println("[BARO] ⚠️ BMP280 no detectado");
    }
}

// ========================================================
//  CALIBRAR CERO BAROMÉTRICO
// ========================================================
void calibrateAltZero(int samples, int delay_ms) {
    if (!bmp_ok) return;

    float acc = 0;
    int ok = 0;

    for (int i = 0; i < samples; i++) {
        float v = bmp.readAltitude(QNH_hPa);
        if (isfinite(v)) {
            acc += v;
            ok++;
        }
        delay(delay_ms);
    }

    if (ok > 0) {
        alt_zero = acc / ok;
        alt_baro_f = 0.0f;
        alt_zero_set = true;

        Serial.printf("[BARO] 🔧 Cero calibrado: %.2f m con %d muestras\n",
                       alt_zero, ok);
    } else {
        Serial.println("[BARO] ❌ Error calibrando cero");
    }
}

// ========================================================
//  ACTUALIZAR ALTITUD BAROMÉTRICA FILTRADA
// ========================================================
void updateAltitudeBaro() {
    if (!bmp_ok || !alt_zero_set) return;

    float newAlt = bmp.readAltitude(QNH_hPa);
    if (!isfinite(newAlt)) return;

    float rel = newAlt - alt_zero;
    float step = fabs(rel - alt_baro_f);

    // rechazo de picos
    if (step > ALT_MAX_STEP) {
        DBG("[BARO] Pico descartado Δ=%.2f\n", step);
        return;
    }

    alt_baro_f = ALT_EMA_ALPHA * rel + (1.0f - ALT_EMA_ALPHA) * alt_baro_f;

    unsigned long now = millis();
    static float prev = 0;

    if (lastAltMs != 0) {
        float dt = (now - lastAltMs) / 1000.0f;
        if (dt > 0.001f) climb_mps = (alt_baro_f - prev) / dt;
    }

    prev = alt_baro_f;
    lastAltMs = now;
}

// ========================================================
//  UPDATE GPS RAW DATA (leer bytes del GPS)
// ========================================================
void updateGPSData() {
    while (SerialGPS.available()) {
        gps.encode(SerialGPS.read());
    }
}

// ========================================================
//  MEDIANA DE 4
// ========================================================
double median4(double a, double b, double c, double d) {
    double arr[4] = {a, b, c, d};
    for (int i = 0; i < 3; i++)
        for (int j = i + 1; j < 4; j++)
            if (arr[j] < arr[i]) {
                double t = arr[i];
                arr[i] = arr[j];
                arr[j] = t;
            }
    return (arr[1] + arr[2]) / 2.0;
}

// ========================================================
//  FILTRO GPS COMPLETO (warm-up + mediana + anti-saltos)
// ========================================================
void filterGPS(double lat, double lon, double alt,
               double &lat_out, double &lon_out, double &alt_out) {

    if (fabs(lat) < 0.0001 && fabs(lon) < 0.0001)
        return;

    lat_window[win_idx] = lat;
    lon_window[win_idx] = lon;
    alt_window[win_idx] = alt;

    win_idx = (win_idx + 1) % 4;

    double lat_m = median4(lat_window[0], lat_window[1], lat_window[2], lat_window[3]);
    double lon_m = median4(lon_window[0], lon_window[1], lon_window[2], lon_window[3]);

    // Primera vez
    if (!have_filter) {
        lat_f = lat_m;
        lon_f = lon_m;
        alt_f = alt;
        have_filter = true;
        gpsWarmup = 0;
        DBG("[GPS] 🆗 Filtro inicializado (warm-up)\n");
    }
    else {
        gpsWarmup++;

        // -----------------------------
        // WARM-UP ADAPTATIVO
        // -----------------------------
        if (gpsWarmup < 200) {
            double dLat = fabs(lat_m - lat_f);
            double dLon = fabs(lon_m - lon_f);

            lat_f = 0.7 * lat_m + 0.3 * lat_f;
            lon_f = 0.7 * lon_m + 0.3 * lon_f;
            alt_f = 0.5 * alt + 0.5 * alt_f;

            if (dLat < 0.00002 && dLon < 0.00002) {
                static int stableCount = 0;
                if (++stableCount >= 8) {
                    gpsWarmup = 9999;
                    DBG("[GPS] ✓ Warm-up completado\n");
                }
            }
        }
        else {
            // -----------------------------
            // FILTRO NORMAL
            // -----------------------------
            if (fabs(lat_m - lat_f) > MAX_JUMP_DEG ||
                fabs(lon_m - lon_f) > MAX_JUMP_DEG)
            {
                DBG("[GPS] ⚠️ Salto descartado\n");
                lat_out = lat_f;
                lon_out = lon_f;
                alt_out = alt_f;
                return;
            }

            float spd = gps.speed.kmph();
            float alpha = (spd < 5.0) ? 0.25 : 0.6;

            lat_f = alpha * lat_m + (1 - alpha) * lat_f;
            lon_f = alpha * lon_m + (1 - alpha) * lon_f;
            alt_f = alpha * alt + (1 - alpha) * alt_f;
        }
    }

    lat_out = lat_f;
    lon_out = lon_f;
    alt_out = alt_f;
}
