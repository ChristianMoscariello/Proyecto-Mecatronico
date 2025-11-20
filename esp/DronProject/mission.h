#ifndef MISSION_H
#define MISSION_H

#include <vector>

struct Coordinate {
    double lat;
    double lon;
};

struct Mission {
    bool loaded = false;
    std::vector<Coordinate> polygon;
    Coordinate home {0,0};
    double altitude = 0;
    double spacing = 0;
    String event_action = "NONE";
};

extern Mission mission;

// Lista de puntos interpolados para la navegación
extern std::vector<Coordinate> pathPoints;
extern int currentWaypoint;

// Funciones
void clearMission();
void generateMissionPath(const Mission &m);

#endif
