#include "telemetry.h"
#include "comms_lora.h"   // para enviar por LoRa
#include "gps_baro.h"       // acceso a lat_f, lon_f, alt_baro_f, etc.

// Flag activable desde main.ino
bool DEBUG_TELEMETRY = false;

static unsigned long lastTelemetryTime = 0;
static const unsigned long TELEMETRY_INTERVAL = 1000;  // 1 Hz

// ---------------------------------------------------------
// Inicialización (opcional)
// ---------------------------------------------------------
void initTelemetry() {
    lastTelemetryTime = 0;
}

// ---------------------------------------------------------
// Enviar JSON de telemetría a GS (NO ACK)
// ---------------------------------------------------------
void sendTelemetry(double lat, double lon, double alt, 
                   double speed, double heading, unsigned long ts)
{
    StaticJsonDocument<256> doc;
    doc["t"] = "TELEMETRY";

    JsonObject d = doc.createNestedObject("d");
    d["lat"]     = lat;
    d["lon"]     = lon;
    d["alt"]     = alt;
    d["speed"]   = speed;
    d["heading"] = heading;
    d["battery"] = 0;     // placeholder sin modificar
    d["status"]  = 0;

    doc["ts"] = ts;

    // 🚀 Enviar sin ACK (tu lógica intacta)
    sendJsonNoAckToGS(doc);

    if (DEBUG_TELEMETRY) {
        Serial.printf("[TL] lat=%.6f lon=%.6f alt=%.2f spd=%.2f hdg=%.2f ts=%lu\n",
                      lat, lon, alt, speed, heading, ts);
    }
}

// ---------------------------------------------------------
// handleTelemetry() → decide CUÁNDO enviar telemetría
// ---------------------------------------------------------
void handleTelemetry() 
{
    if (millis() - lastTelemetryTime < TELEMETRY_INTERVAL)
        return;

    lastTelemetryTime = millis();

    // Datos provenientes del módulo GPS/BARO
    double lat = lat_f;
    double lon = lon_f;
    double alt = alt_baro_f;

    double speed = gps_speed_kmh;     // expuesto por gps_baro
    double heading = gps_heading_deg; // idem

    unsigned long ts = gps_timestamp_unix;  // calculado por gps_baro

    // 🔹 Simulación mantiene interface igual
    if (simulationMode) {
        speed = 5.0;
        heading = 0.0;
    }

    sendTelemetry(lat, lon, alt, speed, heading, ts);
}
