#pragma once
#include <Arduino.h>
#include <mpu9250.h>

// === Variables públicas IMU ===
extern float roll, pitch, yaw_f;
extern float yawGyro, yawMag;

// === Inicialización y calibraciones ===
void initMPU9250();
void calibrateIMU_Static();
void zeroGyroOnARM();

// === Update principal ===
void updateIMU();

// === Estructura de calibración externa ===
struct IMUCalibration;
extern IMUCalibration imuCal;
