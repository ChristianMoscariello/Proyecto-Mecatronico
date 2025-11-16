#include "comms_lora.h"

// ========================================================
// HEADERS (los define tu .ino pero los declaramos extern arriba)
// ========================================================
const char* GS_HDR  = "GS#";
const char* UAV_HDR = "UAV#";
const char* SFX     = "#END";

String loraRxBuf = "";

// ========================================================
// ACK SYSTEM
// ========================================================
#define MAX_PENDING 5

PendingMsg pendingMsgs[MAX_PENDING];
unsigned long ackTimeout = 1500;
int maxRetries = 4;
unsigned long nextMsgCounter = 0;

String generateMsgID() {
    nextMsgCounter++;
    return String(nextMsgCounter);
}

// ========================================================
// INIT LORA
// ========================================================
void initLoRa(int cs, int rst, int irq, long band) {
    LoRa.setPins(cs, rst, irq);

    if (!LoRa.begin(band)) {
        Serial.println("❌ LoRa fallo");
        while(1) delay(1000);
    }

    Serial.println("✅ LoRa inicializado");
}

// ========================================================
// TX SIMPLE (sin ACK)
// ========================================================
void sendJsonNoAckToGS(const JsonDocument& doc) {
    String payload;
    serializeJson(doc, payload);

    String frame = String(UAV_HDR) + payload + SFX;

    LoRa.beginPacket();
    LoRa.print(frame);
    LoRa.endPacket();

    LDBG("[LoRa TX] %s\n", frame.c_str());
}

// ========================================================
// TX CON ACK
// ========================================================
void sendWithAck(const String &jsonPayload, const String &id) {
    String full = jsonPayload;

    if (!jsonPayload.startsWith("UAV#"))
        full = "UAV#" + jsonPayload;

    if (!full.endsWith("#END"))
        full += "#END";

    LoRa.beginPacket();
    LoRa.print(full);
    LoRa.endPacket();

    LDBG("[LoRa TX+ACK] %s\n", full.c_str());

    for (int i = 0; i < MAX_PENDING; i++) {
        if (!pendingMsgs[i].waitingAck) {
            pendingMsgs[i].payload = full;
            pendingMsgs[i].lastSend = millis();
            pendingMsgs[i].retries = maxRetries;
            pendingMsgs[i].waitingAck = true;
            pendingMsgs[i].msgID = id;
            return;
        }
    }

    Serial.println("⚠️ Cola ACK llena");
}

// ========================================================
// RECIBIR ACK
// ========================================================
void handleAck(const String &ackID) {
    for (int i = 0; i < MAX_PENDING; i++) {

        if (pendingMsgs[i].waitingAck && pendingMsgs[i].msgID == ackID) {

            // extraer tipo para log
            String tipo = "desconocido";
            int pos = pendingMsgs[i].payload.indexOf("\"t\":\"");
            if (pos >= 0) {
                int end = pendingMsgs[i].payload.indexOf("\"", pos + 5);
                if (end > pos) tipo = pendingMsgs[i].payload.substring(pos + 5, end);
            }

            Serial.printf("✅ [ACK] ID=%s (Tipo=%s)\n",
                          ackID.c_str(), tipo.c_str());

            pendingMsgs[i].waitingAck = false;
            pendingMsgs[i].payload = "";
            pendingMsgs[i].msgID = "";
            return;
        }
    }

    Serial.printf("⚠️ [ACK desconocido] ID=%s\n", ackID.c_str());
}

// ========================================================
// TX ACK HACIA LA GS
// ========================================================
void sendAckToGS(const String &id) {
    if (id == "") {
        Serial.println("⚠️ ACK sin ID");
        return;
    }

    StaticJsonDocument<128> doc;
    doc["t"] = "ACK";
    JsonObject d = doc.createNestedObject("d");
    d["id"] = id;
    doc["ts"] = millis();

    String p;
    serializeJson(doc, p);

    String msg = String(UAV_HDR) + p + SFX;

    LoRa.beginPacket();
    LoRa.print(msg);
    LoRa.endPacket();

    LDBG("[TX ACK] %s\n", msg.c_str());
}

// ========================================================
// REINTENTOS AUTOMÁTICOS (cola ACK)
// ========================================================
void checkPendingAcks() {
    unsigned long now = millis();

    for (int i = 0; i < MAX_PENDING; i++) {
        if (!pendingMsgs[i].waitingAck) continue;

        if (now - pendingMsgs[i].lastSend > ackTimeout) {

            if (pendingMsgs[i].retries > 0) {

                LoRa.beginPacket();
                LoRa.print(pendingMsgs[i].payload);
                LoRa.endPacket();

                pendingMsgs[i].lastSend = now;
                pendingMsgs[i].retries--;

                LDBG("[ACK Retry] ID=%s (quedan %d)\n",
                     pendingMsgs[i].msgID.c_str(),
                     pendingMsgs[i].retries);

            } else {

                Serial.printf("❌ [ACK perdido] ID=%s descartado\n",
                               pendingMsgs[i].msgID.c_str());

                pendingMsgs[i].waitingAck = false;
                pendingMsgs[i].payload = "";
                pendingMsgs[i].msgID = "";
            }
        }
    }
}

// ========================================================
// FRAME EXTRACTOR
// ========================================================
bool extractNextFrame(String& buf, String& jsonOut, const char* wantedHdr) {
    int posHdr = buf.indexOf(wantedHdr);
    int uavPos = buf.indexOf("UAV#");

    // descartar frames propios
    if (uavPos >= 0 && (uavPos < posHdr || posHdr < 0)) {
        int endU = buf.indexOf(SFX, uavPos);
        if (endU < 0) return false;

        buf.remove(0, endU + strlen(SFX));
        return false;
    }

    if (posHdr < 0) {
        if (buf.length() > 2048) buf.remove(0, buf.length() - 256);
        return false;
    }

    int lbrace = buf.indexOf("{", posHdr);
    if (lbrace < 0) return false;

    int end = buf.indexOf(SFX, lbrace);
    if (end < 0) return false;

    jsonOut = buf.substring(lbrace, end);
    buf.remove(0, end + strlen(SFX));
    jsonOut.trim();

    return true;
}

// ========================================================
// LECTURA LoRa PRINCIPAL
// ========================================================
void handleLoRa() {
    int size = LoRa.parsePacket();
    if (size) {
        while (LoRa.available())
            loraRxBuf += (char)LoRa.read();
    }

    String js;

    // cada JSON válido → enviarlo al parser externo
    while (extractNextFrame(loraRxBuf, js, GS_HDR)) {
        processIncomingJSON(js, true);
    }
}
