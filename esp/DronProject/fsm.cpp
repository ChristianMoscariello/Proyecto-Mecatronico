#include "fsm.h"
#include "navigation.h"
#include "simulator.h"
#include "telemetry.h"
#include "gps_baro.h"

DroneState state = IDLE;
AnalysisResult analysisResult = NONE;

bool loraDisarmCommand = false;
bool loraReturnCommand = false;

static unsigned long stateEntryTime = 0;
static unsigned long analysisStartTime = 0;

void initFSM() {
    state = IDLE;
    analysisResult = NONE;
    loraDisarmCommand = false;
    loraReturnCommand = false;
}

// Reset total
void resetMissionState() {
    mission.loaded = false;
    mission.polygon.clear();
    mission.home = {0,0};
    analysisResult = NONE;
    loraDisarmCommand = false;
    loraReturnCommand = false;
    state = IDLE;
    Serial.println("🔄 FSM → IDLE");
}


// ======================================================
//                UPDATE STATE MACHINE
// ======================================================
void updateStateMachine() {

    // --- Simulación si está activa ---
    if (simulationMode) {
        simulateDroneMotion();
        return;   // no mezclo sim con real
    }

    switch (state) {

        case IDLE:
            break;

        case TAKEOFF:
            if (alt_baro_f >= mission.altitude) {
                Serial.println("⛅ Altitud alcanzada → NAVIGATE");
                state = NAVIGATE;
                stateEntryTime = millis();
            }
            break;

        case NAVIGATE: {

            if (currentWaypoint >= pathPoints.size()) {
                Serial.println("🏁 Ruta completa → RETURN_HOME");
                state = RETURN_HOME;
                break;
            }

            Coordinate target = pathPoints[currentWaypoint];
            double dist = haversineDistance(lat_f, lon_f, target.lat, target.lon);

            if (dist < 2.0) {
                Serial.printf("📍 WP %d alcanzado\n", currentWaypoint);
                sendStableToRPi(target);
                state = STABILIZE;
                stateEntryTime = millis();
            }
            else navigateTo(target);

        } break;

        case STABILIZE:
            if (millis() - stateEntryTime > 300) {
                Serial.println("📷 Esperando RPi...");
                analysisStartTime = millis();
                state = WAIT_ANALYSIS;
            }
            break;

        case WAIT_ANALYSIS:

            // Interrupciones por LoRa
            if (loraReturnCommand) {
                loraReturnCommand = false;
                Serial.println("⚠️ RETURN recibido → HOME");
                state = RETURN_HOME;
                break;
            }
            if (loraDisarmCommand) {
                loraDisarmCommand = false;
                Serial.println("🛑 DISARM → abortado");
                resetMissionState();
                break;
            }

            // Respuesta RPi
            if (analysisResult != NONE) {
                if (analysisResult == GO) {
                    currentWaypoint++;
                    state = NAVIGATE;
                }
                else if (analysisResult == FIRE || analysisResult == PERSON) {
                    if (mission.event_action == "RETURN")
                        state = RETURN_HOME;
                    else
                        currentWaypoint++;

                    state = NAVIGATE;
                }
                analysisResult = NONE;
            }

            // Timeout
            else if (millis() - analysisStartTime > ANALYSIS_TIMEOUT) {
                Serial.println("⌛ Timeout RPi → seguir");
                currentWaypoint++;
                state = NAVIGATE;
            }

            break;

        case RETURN_HOME: {

            double distHome = haversineDistance(lat_f, lon_f,
                                                mission.home.lat,
                                                mission.home.lon);

            if (distHome < 2.0) {
                Serial.println("🛬 HOME → Landing");
                state = LAND;
            }
            else navigateTo(mission.home);

        } break;

        case LAND:
            if (alt_baro_f < 1.0) {
                Serial.println("🟢 Aterrizado");
                state = COMPLETE;
            }
            break;

        case COMPLETE:
            Serial.println("✔ Misión finalizada");
            resetMissionState();
            break;
    }
}
