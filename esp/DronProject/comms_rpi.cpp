#include "comms_rpi.h"

// ========================================================
// CONFIG
// ========================================================
HardwareSerial SerialRPI(1);
const unsigned long RPI_TIMEOUT_MS = 50;

// FRAMES
const char* RPI_HDR = "RPI#";
const char* UAV_HDR = "UAV#";
const char* END_TAG = "#END";

String rpiRxBuffer = "";

// ========================================================
// INIT
// ========================================================
void initRPI() {
    SerialRPI.begin(9600, SERIAL_8N1, RPI_RX, RPI_TX);
    SerialRPI.setTimeout(RPI_TIMEOUT_MS);
    Serial.println("✅ RPI UART inicializado");

    rpiRxBuffer.reserve(2048);
}

// ========================================================
// SEND JSON A RPI
// ========================================================
void sendJsonToRPI(const JsonDocument &doc) {
    String payload;
    serializeJson(doc, payload);

    String frame = String(UAV_HDR) + payload + END_TAG;
    SerialRPI.print(frame);

    RDBG("[RPI TX] %s\n", frame.c_str());
}

// ========================================================
// FRAME PARSER (misma lógica que LoRa pero independiente)
// ========================================================
bool extractRpiFrame(String &buf, String &jsonOut, const char* wantedHdr) {
    int posHdr = buf.indexOf(wantedHdr);

    // descartar frames propios UAV#
    int uavPos = buf.indexOf("UAV#");
    if (uavPos >= 0 && (uavPos < posHdr || posHdr < 0)) {
        int endU = buf.indexOf(END_TAG, uavPos);
        if (endU < 0) return false;

        buf.remove(0, endU + strlen(END_TAG));
        return false;
    }

    if (posHdr < 0) {
        if (buf.length() > 2048) buf.remove(0, buf.length() - 256);
        return false;
    }

    int brace = buf.indexOf("{", posHdr);
    if (brace < 0) return false;

    int end = buf.indexOf(END_TAG, brace);
    if (end < 0) return false;

    jsonOut = buf.substring(brace, end);
    buf.remove(0, end + strlen(END_TAG));

    jsonOut.trim();
    return true;
}

// ========================================================
// RX LOOP
// ========================================================
void handleSerialRPI() {
    while (SerialRPI.available()) {
        char c = SerialRPI.read();
        rpiRxBuffer += c;
    }

    String json;

    while (extractRpiFrame(rpiRxBuffer, json, RPI_HDR)) {
        RDBG("[RPI RX] %s\n", json.c_str());
        processIncomingJSON(json, false);
    }
}
