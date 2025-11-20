#include "simulator.h"
#include "fsm.h"
#include "gps_baro.h"
#include "mission.h"
#include "navigation.h"

void simulateDroneMotion() {

    static unsigned long lastStepTime = 0;
    if (millis() - lastStepTime < 600) return;
    lastStepTime = millis();

    switch (state) {

        case TAKEOFF:
            alt_baro_f += 0.4;
            if (alt_baro_f >= mission.altitude) {
                alt_baro_f = mission.altitude;
                state = NAVIGATE;
            }
            break;

        case NAVIGATE: {
            if (currentWaypoint >= pathPoints.size()) {
                state = RETURN_HOME;
                break;
            }

            Coordinate t = pathPoints[currentWaypoint];
            double d = haversineDistance(lat_f, lon_f, t.lat, t.lon);

            lat_f += (t.lat - lat_f) * 0.25;
            lon_f += (t.lon - lon_f) * 0.25;

            if (d < 1.5) {
                sendStableToRPi(t);
                state = STABILIZE;
            }
        } break;

        case STABILIZE:
            state = WAIT_ANALYSIS;
            break;

        case WAIT_ANALYSIS:

            if (analysisResult == GO) {
                currentWaypoint++;
                analysisResult = NONE;
                state = NAVIGATE;
            }

            else if (analysisResult == FIRE || analysisResult == PERSON) {
                if (mission.event_action == "RETURN")
                    state = RETURN_HOME;
                else
                    currentWaypoint++;
                analysisResult = NONE;
                state = NAVIGATE;
            }

            break;

        case RETURN_HOME:
            lat_f += (mission.home.lat - lat_f) * 0.25;
            lon_f += (mission.home.lon - lon_f) * 0.25;

            if (haversineDistance(lat_f, lon_f,
                                  mission.home.lat, mission.home.lon) < 1.5) {
                state = LAND;
            }
            break;

        case LAND:
            alt_baro_f = max(0.0f, alt_baro_f - 0.15f);
            if (alt_baro_f <= 0.2f) {
                state = COMPLETE;
            }
            break;

        case COMPLETE:
            resetMissionState();
            break;

        default:
            break;
    }
}
