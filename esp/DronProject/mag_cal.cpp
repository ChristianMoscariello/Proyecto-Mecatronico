#include "mag_cal.h"
#include <Arduino.h>
#include <Wire.h>
#include "imu.h"     // ← para imuCal, readQMC, etc.

#define MAG_DEBUG 2
/* 
  0 = no imprime nada
  1 = imprime solo resultados importantes
  2 = imprime todo lo técnico (rango, min/max, etc.)
*/

// =========================
//   CONSTANTES / LIMITES
// =========================
#define MAX_MAG_SAMPLES   1200    // ~10s a 50-100 Hz
#define MIN_RANGE_uT      5.0f    // rango mínimo aceptable por eje
#define MAG_FIELD_REF_uT  50.0f   // referencia global
#define MAG_RECAL_THRESH  0.35f   // 35% diferencia → warning

// =========================
//   BUFFERS Y ESTADO
// =========================
struct MagSample {
    float x, y, z;
};

static MagSample magBuff[MAX_MAG_SAMPLES];
static int      magCount          = 0;
static bool     magCalActivePRO   = false;
static unsigned long magStart     = 0;
static unsigned long magDuration  = 0;

// =========================
//   REFERENCIAS EXTERNAS
// =========================
extern IMUCalibration imuCal;
extern bool readQMC5883L(float&, float&, float&, float&);

// =========================
//   HELPERS DEBUG
// =========================
#define DBG_L1(...)  if (MAG_DEBUG >= 1) { Serial.printf(__VA_ARGS__); }
#define DBG_L2(...)  if (MAG_DEBUG >= 2) { Serial.printf(__VA_ARGS__); }

// =========================
//   INICIAR CALIBRACIÓN
// =========================
void startMagCalibrationPRO(unsigned long durationMs) {
    magCalActivePRO = true;
    magCount        = 0;
    magStart        = millis();
    magDuration     = durationMs;

    DBG_L1("\n🧭 [MAG PRO] Iniciando calibración 3D (%lu ms)\n", durationMs);

    // Aviso a GS
    StaticJsonDocument<192> j;
    j["t"] = "MAG_CAL_PROGRESS";
    JsonObject d = j.createNestedObject("d");
    d["phase"] = "START_3D";
    d["duration_ms"] = (int)durationMs;
    j["ts"] = (int)millis();
    sendJsonNoAckToGS(j);
}

// =========================
//      LOOP DE CALIBRACIÓN
// =========================
void updateMagCalibrationPRO() {
    if (!magCalActivePRO) return;

    float mx, my, mz, dummy;
    if (!readQMC5883L(mx, my, mz, dummy)) {
        readQMC5883L(mx, my, mz, dummy); // última válida
    }

    if (magCount < MAX_MAG_SAMPLES) {
        magBuff[magCount++] = { mx, my, mz };
    }

    static unsigned long lastPrint = 0;
    if (millis() - lastPrint > 2000) {
        lastPrint = millis();
        DBG_L2("[MAG PRO] samples=%d  elapsed=%lu ms\n", magCount, millis() - magStart);
    }

    if (millis() - magStart > magDuration) {
        magCalActivePRO = false;
        computeMagCalibrationPRO();
    }
}

// =========================
//       COMPUTE FINAL
// =========================
void computeMagCalibrationPRO() {
    DBG_L1("\n📊 [MAG PRO] Procesando %d muestras...\n", magCount);

    if (magCount < 200) {
        DBG_L1("❌ Pocas muestras. Repetir calibración.\n");
        return;
    }

    // Min/max por eje
    float minX =  1e6, minY =  1e6, minZ =  1e6;
    float maxX = -1e6, maxY = -1e6, maxZ = -1e6;

    for (int i = 0; i < magCount; i++) {
        minX = min(minX, magBuff[i].x);
        minY = min(minY, magBuff[i].y);
        minZ = min(minZ, magBuff[i].z);

        maxX = max(maxX, magBuff[i].x);
        maxY = max(maxY, magBuff[i].y);
        maxZ = max(maxZ, magBuff[i].z);
    }

    float rangeX = maxX - minX;
    float rangeY = maxY - minY;
    float rangeZ = maxZ - minZ;

    DBG_L2("Rangos:\n X: %.3f  Y: %.3f  Z: %.3f\n", rangeX, rangeY, rangeZ);

    // === Validez ===
    bool badX = (rangeX < MIN_RANGE_uT);
    bool badY = (rangeY < MIN_RANGE_uT);
    bool badZ = (rangeZ < MIN_RANGE_uT);

    if (badX || badY || badZ) {
        DBG_L1("⚠️ Rango insuficiente. Solo bias corregido.\n");
        imuCal.magBias[0] = (maxX + minX)*0.5f;
        imuCal.magBias[1] = (maxY + minY)*0.5f;
        imuCal.magBias[2] = (maxZ + minZ)*0.5f;
        imuCal.magScale[0] = imuCal.magScale[1] = imuCal.magScale[2] = 1.0f;

        printMagCalibration();
        return;
    }

    // === bias ===
    float bx = (maxX + minX)*0.5f;
    float by = (maxY + minY)*0.5f;
    float bz = (maxZ + minZ)*0.5f;

    // === soft iron simple ===
    float avgR = (rangeX + rangeY + rangeZ)/3.0f;
    float sx = avgR / rangeX;
    float sy = avgR / rangeY;
    float sz = avgR / rangeZ;

    imuCal.magBias[0]  = bx;
    imuCal.magBias[1]  = by;
    imuCal.magBias[2]  = bz;

    imuCal.magScale[0] = sx;
    imuCal.magScale[1] = sy;
    imuCal.magScale[2] = sz;

    DBG_L1("✅ Bias:  %.3f  %.3f  %.3f\n", bx, by, bz);
    DBG_L1("   Scale: %.3f  %.3f  %.3f\n", sx, sy, sz);

    // report to GS
    StaticJsonDocument<256> j;
    j["t"] = "MAG_CAL_RESULT";
    JsonObject d = j.createNestedObject("d");

    JsonObject bias = d.createNestedObject("bias");
    bias["x"] = bx; bias["y"] = by; bias["z"] = bz;

    JsonObject sc = d.createNestedObject("scale");
    sc["x"] = sx; sc["y"] = sy; sc["z"] = sz;

    d["samples"] = magCount;
    d["result"]  = "OK_3D";

    j["ts"] = (int)millis();
    sendJsonNoAckToGS(j);

    printMagCalibration();
}

// =========================
//   CALIDAD DEL CAMPO
// =========================
void checkMagQualitySuggest() {
    static unsigned long lastCheck = 0;
    static int badCount = 0;
    static unsigned long lastBadTime = 0;

    if (millis() - lastCheck < 3000) return;
    lastCheck = millis();

    float mx, my, mz, dummy;
    readQMC5883L(mx, my, mz, dummy);

    float mag = sqrtf(mx*mx + my*my + mz*mz);
    float dev = fabsf(mag - MAG_FIELD_REF_uT) / MAG_FIELD_REF_uT;

    if (dev > MAG_RECAL_THRESH) {
        badCount++;
        lastBadTime = millis();
    } else badCount = 0;

    if (badCount >= 3) {
        DBG_L1("⚠️ MAG desviado (%.2f) → sugerir recalibración\n", dev);

        StaticJsonDocument<192> j;
        j["t"] = "MAG_RECAL_SUGGEST";
        JsonObject d = j.createNestedObject("d");
        d["deviation"] = dev;
        d["field"] = mag;
        d["quality"] = "BAD";
        j["ts"] = (int)millis();
        sendJsonNoAckToGS(j);

        badCount = 0;
    }
}

// =========================
//   PRINT DE CALIBRACIÓN
// =========================
void printMagCalibration() {
    DBG_L1("\n=== Calibración Magnética ===\n");
    DBG_L1("Bias:  X=%.3f  Y=%.3f  Z=%.3f\n",
           imuCal.magBias[0], imuCal.magBias[1], imuCal.magBias[2]);

    DBG_L1("Scale: X=%.4f  Y=%.4f  Z=%.4f\n",
           imuCal.magScale[0], imuCal.magScale[1], imuCal.magScale[2]);
}
