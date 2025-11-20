#ifndef UTILS_H
#define UTILS_H

#include <Arduino.h>

double deg2rad(double d);
double rad2deg(double r);

unsigned long toUnixTime(int y, int m, int d,
                         int h, int min, int s);

bool isBadFloat(float v);

#endif
