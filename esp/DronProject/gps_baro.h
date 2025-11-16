#pragma once
#include <Arduino.h>
#include <TinyGPS++.h>
#include <Adafruit_BMP280.h>

// Extern de GPS y BMP que ya están creados en tu .ino
extern TinyGPSPlus gps;
extern Adafruit_BMP280 bmp;
extern bool bmp_ok;

// ========================================================
// Variables filtradas disponibles para todo el dron
// ========================================================
extern double lat_f;
extern double lon_f;
extern double alt_f;       // altitud GPS filtrada
extern float  alt_baro_f;  // altitud barométrica filtrada
extern float  climb_mps;   // razón de ascenso

// ========================================================
// API del módulo GPS + BARO
// ========================================================
void initBaro();                    // inicializa BMP, sampling, etc.
void calibrateAltZero(int samples = 60, int delay_ms = 20);
void updateAltitudeBaro();

void updateGPSData();              // lee byte a byte → gps.encode()
void filterGPS(double lat, double lon, double alt,
               double &lat_out, double &lon_out, double &alt_out);

// Utilidades (por si se necesitan fuera)
double median4(double a, double b, double c, double d);
