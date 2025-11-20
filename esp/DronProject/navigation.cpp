#include "navigation.h"
#include "gps_baro.h"

int currentWaypoint = 0;
std::vector<Coordinate> pathPoints;

void navigateTo(const Coordinate& target) {
    double bearing = computeBearing(lat_f, lon_f, target.lat, target.lon);
    double dist = haversineDistance(lat_f, lon_f, target.lat, target.lon);

    Serial.printf("🧭 NAV → brg=%.1f°, dist=%.1f\n", bearing, dist);

    if (dist > 0.3) {
        double alpha = 0.05;
        lat_f += (target.lat - lat_f) * alpha;
        lon_f += (target.lon - lon_f) * alpha;
    }
}
