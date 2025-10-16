// Usaremos el puerto Serial2 del ESP32
HardwareSerial& RaspiSerial = Serial2;

void setup() {
  // Inicia el monitor serie para depuración (USB)
  Serial.begin(115200);

  // Inicia la comunicación con la Raspberry Pi (velocidad debe coincidir)
  // Reemplaza esto:
  // RaspiSerial.begin(9600);

  // Por esto:
  // begin(baudrate, protocol, RX_PIN, TX_PIN)
  RaspiSerial.begin(9600, SERIAL_8N1, 16, 17);

  Serial.println("ESP32 listo para comunicación bidireccional...");
}

void loop() {
  // Si hay un mensaje de la Pi...
  if (RaspiSerial.available() > 0) {

    // Lee el mensaje entrante
    String mensajeRecibido = RaspiSerial.readStringUntil('\n');

    // Muestra el mensaje en el monitor de tu PC
    Serial.print("Mensaje recibido de la Pi: ");
    Serial.println(mensajeRecibido);

    // --- NUEVA LÍNEA ---
    // Envía una respuesta de vuelta a la Raspberry Pi
    RaspiSerial.println("recibí tu mensaje");
  }
}