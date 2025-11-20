#include "imu.h"
#include <Wire.h>

// ====== Variables accesibles globalmente ======
float roll = 0, pitch = 0, yaw_f = 0;
float yawGyro = 0, yawMag = 0;

// ====== Suavizados (EMA) ======
#define ALPHA_ACC  0.15f
#define ALPHA_GYRO 0.15f
#define ALPHA_MAG  0.20f
#define ALPHA_YAW  0.05f

static float ax_f=0, ay_f=0, az_f=0;
static float gx_f=0, gy_f=0, gz_f=0;
static float mx_f=0, my_f=0, mz_f=0;

// ====== Referencias externas ======
extern bfs::Mpu9250 mpu;
extern IMUCalibration imuCal;

extern const float MAG_DECLINATION_DEG;

// ============================================================
// 🔧 Inicialización optimizada MPU9250
// ============================================================
void initMPU9250() {
    Serial.println("🔧 Inicializando MPU9250…");

    Wire.begin();               // 🔥 Consolidado
    Wire.setClock(400000);      // 🔥 I2C rápido

    if (!mpu.Begin()) {
        Serial.println("❌ Error iniciando MPU9250");
        return;
    }

    mpu.ConfigAccelRange(bfs::Mpu9250::ACCEL_RANGE_4G);
    mpu.ConfigGyroRange (bfs::Mpu9250::GYRO_RANGE_500DPS);
    mpu.ConfigDlpfBandwidth(bfs::Mpu9250::DLPF_BANDWIDTH_41HZ);

    mpu.ConfigSrd(9);   // 🔥 100 Hz, antes 19 → 50 Hz

    Serial.println("✅ MPU9250 listo (100 Hz)");
}


// ============================================================
// 🔧 Calibración estática ACC+GYRO (con no-blocking)
// ============================================================
void calibrateIMU_Static() {
    Serial.println("🧪 Calibración estática ACC+GYRO (quieto)");

    const int N = 300;
    float ax=0, ay=0, az=0;
    float gx=0, gy=0, gz=0;
    int count = 0;

    unsigned long last = millis();

    while (count < N) {
        if (millis() - last >= 5) {
            last = millis();
            if (mpu.Read()) {
                ax += mpu.accel_x_mps2();
                ay += mpu.accel_y_mps2();
                az += mpu.accel_z_mps2();
                gx += mpu.gyro_x_radps();
                gy += mpu.gyro_y_radps();
                gz += mpu.gyro_z_radps();
                count++;
            }
        }
    }

    imuCal.accelBias[0] = ax/N;
    imuCal.accelBias[1] = ay/N;
    imuCal.accelBias[2] = (az/N) - 9.80665f;

    imuCal.gyroBias[0]  = gx/N;
    imuCal.gyroBias[1]  = gy/N;
    imuCal.gyroBias[2]  = gz/N;

    Serial.println("✅ Calibración IMU estática lista");
}

// ============================================================
// 🔧 Zero Gyro on ARM
// ============================================================
void zeroGyroOnARM() {
    Serial.println("🟢 Zero GYRO en ARM");

    const int N = 200;
    float gx=0, gy=0, gz=0;
    int count = 0;

    unsigned long last = millis();

    while (count < N) {
        if (millis() - last >= 3) {
            last = millis();
            if (mpu.Read()) {
                gx += mpu.gyro_x_radps();
                gy += mpu.gyro_y_radps();
                gz += mpu.gyro_z_radps();
                count++;
            }
        }
    }

    imuCal.gyroBias[0] = gx/N;
    imuCal.gyroBias[1] = gy/N;
    imuCal.gyroBias[2] = gz/N;

    Serial.println("✅ Gyro centrado para ARM");
}


// ============================================================
// 🔥 Función YAW — tilt compensation + complementary
// ============================================================
static float tiltCompensatedYaw(float rollDeg, float pitchDeg,
                                float mx, float my, float mz)
{
    float roll = rollDeg * DEG_TO_RAD;
    float pitch = pitchDeg * DEG_TO_RAD;

    float sinR = sinf(roll),  cosR = cosf(roll);
    float sinP = sinf(pitch), cosP = cosf(pitch);

    float mx_c = mx*cosP + mz*sinP;
    float my_c = mx*sinR*sinP + my*cosR - mz*sinR*cosP;

    float heading = atan2f(-my_c, mx_c) * RAD_TO_DEG;
    if (heading < 0) heading += 360.0f;

    return heading;
}

static void fuseYaw(float gz, float dt, float yawMagIn) {
    static float yawInt = 0;

    yawInt += gz * RAD_TO_DEG * dt;

    if (yawInt >= 360) yawInt -= 360;
    else if (yawInt < 0) yawInt += 360;

    yawGyro = yawInt;
    yawGyro = (1-ALPHA_YAW)*yawGyro + ALPHA_YAW*yawMagIn;

    yaw_f = yawGyro + MAG_DECLINATION_DEG;
    if (yaw_f >= 360) yaw_f -= 360;
    else if (yaw_f < 0) yaw_f += 360;
}

// ============================================================
// 🔄 UPDATE PRINCIPAL IMU (100 Hz)
// ============================================================
void updateIMU() {
    if (!mpu.Read()) return;

    static unsigned long last = millis();
    float dt = (millis() - last) * 0.001f;
    last = millis();

    // === Lecturas crudas ===
    float ax = mpu.accel_x_mps2() - imuCal.accelBias[0];
    float ay = mpu.accel_y_mps2() - imuCal.accelBias[1];
    float az = mpu.accel_z_mps2() - imuCal.accelBias[2];

    float gx = mpu.gyro_x_radps() - imuCal.gyroBias[0];
    float gy = mpu.gyro_y_radps() - imuCal.gyroBias[1];
    float gz = mpu.gyro_z_radps() - imuCal.gyroBias[2];

    float mx = (mpu.mag_x_ut() - imuCal.magBias[0]) * imuCal.magScale[0];
    float my = (mpu.mag_y_ut() - imuCal.magBias[1]) * imuCal.magScale[1];
    float mz = (mpu.mag_z_ut() - imuCal.magBias[2]) * imuCal.magScale[2];

    // === Suavizado ===
    ax_f = (1-ALPHA_ACC)*ax_f + ALPHA_ACC*ax;
    ay_f = (1-ALPHA_ACC)*ay_f + ALPHA_ACC*ay;
    az_f = (1-ALPHA_ACC)*az_f + ALPHA_ACC*az;

    gx_f = (1-ALPHA_GYRO)*gx_f + ALPHA_GYRO*gx;
    gy_f = (1-ALPHA_GYRO)*gy_f + ALPHA_GYRO*gy;
    gz_f = (1-ALPHA_GYRO)*gz_f + ALPHA_GYRO*gz;

    mx_f = (1-ALPHA_MAG)*mx_f + ALPHA_MAG*mx;
    my_f = (1-ALPHA_MAG)*my_f + ALPHA_MAG*my;
    mz_f = (1-ALPHA_MAG)*mz_f + ALPHA_MAG*mz;

    // === Roll y Pitch ===
    roll  = atan2f(ay_f, az_f) * RAD_TO_DEG;
    pitch = atan2f(-ax_f, sqrtf(ay_f*ay_f + az_f*az_f)) * RAD_TO_DEG;

    // === Yaw (mag + gyro fusion) ===
    yawMag = tiltCompensatedYaw(roll, pitch, mx_f, my_f, mz_f);
    fuseYaw(gz_f, dt, yawMag);
}
