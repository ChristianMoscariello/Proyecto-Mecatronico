#ifndef FSM_H
#define FSM_H

#include <Arduino.h>
#include "mission.h"   // definición de Mission, Coordinate
#include "comms_rpi.h"

// Estados del dron
enum DroneState {
    IDLE,
    TAKEOFF,
    NAVIGATE,
    STABILIZE,
    WAIT_ANALYSIS,
    RETURN_HOME,
    LAND,
    COMPLETE
};

extern DroneState state;

// Resultados de análisis (desde RPi)
enum AnalysisResult { NONE, GO, FIRE, PERSON };
extern AnalysisResult analysisResult;

// Inicialización y actualización
void initFSM();
void updateStateMachine();
void resetMissionState();

// Flags desde LoRa
extern bool loraDisarmCommand;
extern bool loraReturnCommand;

#endif
