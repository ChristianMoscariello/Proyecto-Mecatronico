#ifndef NAVIGATION_H
#define NAVIGATION_H

#include <Arduino.h>
#include "mission.h"

void navigateTo(const Coordinate& target);
double haversineDistance(double lat1, double lon1, double lat2, double lon2);
double computeBearing(double lat1, double lon1, double lat2, double lon2);

extern int currentWaypoint;
extern std::vector<Coordinate> pathPoints;

#endif
