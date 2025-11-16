#include "utils.h"
#include <math.h>

// ============================================================
//                   Conversión de ángulos
// ============================================================
double deg2rad(double d) { return d * M_PI / 180.0; }
double rad2deg(double r) { return r * 180.0 / M_PI; }

// ============================================================
//                     Unix time desde GPS
// ============================================================
unsigned long toUnixTime(int y,int m,int d,
                         int h,int min,int s) {

    if (m <= 2) { y -= 1; m += 12; }
    long a = y / 100;
    long b = 2 - a + a / 4;

    long days = (long)(365.25 * (y + 4716))
              + (long)(30.6001 * (m + 1))
              + d + b - 1524.5;

    unsigned long ts = (days - 2440588) * 86400UL
                     + h * 3600UL
                     + min * 60UL
                     + s;

    return ts;
}

// ============================================================
//                     Validación float
// ============================================================
bool isBadFloat(float v) {
    return isnan(v) || isinf(v);
}
