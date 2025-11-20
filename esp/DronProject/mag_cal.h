#pragma once
#include <Arduino.h>

void startMagCalibrationPRO(unsigned long durationMs);
void updateMagCalibrationPRO();
void computeMagCalibrationPRO();
void checkMagQualitySuggest();
void printMagCalibration();
