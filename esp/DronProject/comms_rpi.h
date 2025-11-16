#pragma once
#include <Arduino.h>
#include <ArduinoJson.h>

// ========================================================
// CONFIG
// ========================================================
#define RPI_DEBUG 0
// 0 = nada
// 1 = logs importantes
// 2 = verbose extremo

#define RDBG(...)  if (RPI_DEBUG >= 1) { Serial.printf(__VA_ARGS__); }
#define RDBG2(...) if (RPI_DEBUG >= 2) { Serial.printf(__VA_ARGS__); }

// ========================================================
// CONSTANTES
// ========================================================
extern HardwareSerial SerialRPI;
extern const unsigned long RPI_TIMEOUT_MS;

// Frame tags
extern const char* RPI_HDR;    // "RPI#"
extern const char* UAV_HDR;    // "UAV#"
extern const char* END_TAG;    // "#END"

// ========================================================
// ESTADO GLOBAL
// ========================================================
extern String rpiRxBuffer;

// ========================================================
// API PRINCIPAL
// ========================================================

// init UART y buffer
void initRPI();

// lectura no bloqueante
void handleSerialRPI();

// envío simple
void sendJsonToRPI(const JsonDocument &doc);

// frame parser reutilizable
bool extractRpiFrame(String &buf, String &jsonOut, const char* wantedHdr);

// json parser externo (lo implementa tu .ino)
void processIncomingJSON(const String &jsonIn, bool fromGSorRPI);
