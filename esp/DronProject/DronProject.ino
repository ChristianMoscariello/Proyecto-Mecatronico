// ============================================================================
// DRON BUSCADOR – MAIN
// Sistema modularizado por carpetas
// ============================================================================

// ---------- MÓDULOS DE SENSORES ----------
#include "imu.h"
#include "mag_cal.h"
#include "gps_baro.h"

// ---------- COMUNICACIÓN ----------
#include "comms_lora.h"
#include "comms_rpi.h"
#include "telemetry.h"

// ---------- LÓGICA DE NAVEGACIÓN Y MISIÓN ----------
#include "mission.h"
#include "navigation.h"
#include "simulation.h"
#include "fsm.h"

// ---------- UTILIDADES ----------
#include "utils.h"


// ============================================================================
// SETUP PRINCIPAL
// ============================================================================
void setup() {
    Serial.begin(115200);
    delay(300);

    Serial.println("======================================");
    Serial.println("🚀 DRON BUSCADOR - Sistema Modular");
    Serial.println("======================================");

    // ------------------------- UART GPS -------------------------
    initGPS();        // (GPS_RX, GPS_TX quedan en gps_baro)

    // ------------------------- UART RPI -------------------------
    initRPI();        // UART1

    // ------------------------- LoRa -----------------------------
    initLoRa();       // CS, RST, IRQ definidos en comms_lora

    // ------------------------- BMP280 ---------------------------
    initBarometer();  // bmp.begin + configuración

    // ------------------------- IMU + MAG ------------------------
    initMPU();            // BolderFlight + biases
    loadIMUCalibration(); // NVS
    calibrateIMU_Static();

    // ------------------------- PID & TRIMS ----------------------
    loadTrims();
    loadPID();

    // ------------------------- FSM inicial -----------------------
    initFSM();

    // ------------------------- DEBUG -----------------------------
    printMagCalibration();
    Serial.println("Comandos por USB:");
    Serial.println("  C -> Calibración MAG");
    Serial.println("  P -> Print calibración");
    Serial.println("  M -> Medir magnetómetro");
}


// ============================================================================
// LOOP PRINCIPAL
// ============================================================================
void loop() 
{
    // ---------- Sensores ----------
    updateGPS();
    updateBarometer();
    updateIMU();
    updateMagCalibrationPRO();      // si está activa
    checkMagQualitySuggest();       // sugiere recalibración

    // ---------- Comunicaciones ----------
    handleLoRa();
    handleSerialRPI();
    checkPendingAcks();

    // ---------- Telemetría ----------
    handleTelemetry();

    // ---------- Máquina de estados ----------
    updateFSM();

    // ---------- Simulación ----------
    simulateDrone();   // SOLO en modo sim

    // ---------- Comandos manuales ----------
    if (Serial.available()) {
        char c = Serial.read();
        if (c == 'C') startMagCalibrationPRO(20000);
        if (c == 'P') printMagCalibration();
        if (c == 'M') printRawMag();
    }
}
