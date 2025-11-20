#include "mission.h"
#include "navigation.h"

Mission mission;

std::vector<Coordinate> pathPoints;
int currentWaypoint = 0;

// Reset completo solo de la misión, sin tocar FSM
void clearMission() {
    mission.loaded = false;
    mission.polygon.clear();
    pathPoints.clear();
    currentWaypoint = 0;
}


// ========================================================
//         GENERAR PUNTOS DE RECORRIDO (interpolación)
// ========================================================
void generateMissionPath(const Mission &m) {
    pathPoints.clear();
    currentWaypoint = 0;

    if (m.polygon.size() < 1) return;

    Coordinate start = m.home;
    Coordinate end   = m.polygon[0];

    double totalDist = haversineDistance(start.lat, start.lon,
                                         end.lat, end.lon);

    int steps = (m.spacing > 0.5)
                ? (int)(totalDist / m.spacing)
                : 1;

    if (steps < 1) steps = 1;

    for (int i = 0; i <= steps; i++) {
        double t = (double)i / (double)steps;
        Coordinate pt;
        pt.lat = start.lat + (end.lat - start.lat) * t;
        pt.lon = start.lon + (end.lon - start.lon) * t;
        pathPoints.push_back(pt);
    }

    Serial.printf(
        "🧭 Path generado: %u puntos (dist=%.1f m, spacing=%.1f)\n",
        pathPoints.size(), totalDist, m.spacing
    );
}
