#ifndef TELEMETRY_H
#define TELEMETRY_H

#include <Arduino.h>
#include <ArduinoJson.h>

// --- Configuración ---
void initTelemetry();   // opcional
void handleTelemetry();  // llamada desde loop()

// --- Función base (usada por otros módulos) ---
void sendTelemetry(double lat, double lon, double alt,
                   double speed, double heading, unsigned long ts);

// --- Flags de debug ---
extern bool DEBUG_TELEMETRY;

#endif
