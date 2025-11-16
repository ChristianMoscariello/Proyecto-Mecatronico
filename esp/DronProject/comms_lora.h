#pragma once
#include <Arduino.h>
#include <LoRa.h>
#include <ArduinoJson.h>

// ========================================================
// FRAMING CONSTANTS
// ========================================================
extern const char* GS_HDR;
extern const char* UAV_HDR;
extern const char* SFX;

extern String loraRxBuf;

// ========================================================
// ACK + QUEUE STRUCTURES
// ========================================================
struct PendingMsg {
    String payload;
    unsigned long lastSend;
    int retries;
    bool waitingAck;
    String msgID;
};

extern PendingMsg pendingMsgs[];
extern unsigned long ackTimeout;
extern int maxRetries;

// ========================================================
// API COMMS LoRa
// ========================================================

// Init
void initLoRa(int cs, int rst, int irq, long band);

// Main RX processing
void handleLoRa();

// TX without ACK
void sendJsonNoAckToGS(const JsonDocument& doc);

// ACK TX
void sendAckToGS(const String& id);

// TX with guaranteed ACK
void sendWithAck(const String &jsonPayload, const String &id);

// ACK re-transmit manager
void checkPendingAcks();

// Frame extractor
bool extractNextFrame(String& buf, String& jsonOut, const char* wantedHdr);

// For external JSON processor
void processIncomingJSON(const String &jsonIn, bool fromGS);

// ID generator (expuesto)
String generateMsgID();

// ========================================================
// DEBUG ENABLE
// ========================================================
#define LORA_DEBUG 0
// 0 = nada
// 1 = eventos importantes
// 2 = verbose extremo

#define LDBG(...) if (LORA_DEBUG >= 1) { Serial.printf(__VA_ARGS__); }
#define LDBG2(...) if (LORA_DEBUG >= 2) { Serial.printf(__VA_ARGS__); }
