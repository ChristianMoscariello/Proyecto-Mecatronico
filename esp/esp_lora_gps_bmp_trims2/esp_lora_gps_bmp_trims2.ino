#include <SPI.h>
#include <LoRa.h>
#include <TinyGPS++.h>
#include <ArduinoJson.h>
#include <Adafruit_BMP280.h>
#include <Preferences.h>

// =============================
// Configuración GPS 
// =============================
#define GPS_RX 16     
#define GPS_TX 17     
HardwareSerial SerialGPS(2);
TinyGPSPlus gps;

// =============================
// Configuración LoRa
// =============================
#define LORA_CS 5
#define LORA_RST 14
#define LORA_IRQ 27
#define LORA_BAND 433E6

// =============================
// Configuración Raspberry
// =============================
#define RPI_RX  4     
#define RPI_TX  2     
HardwareSerial SerialRPI(1);    // UART1 para Raspberry

// =============================
// Configuración BMP280
// =============================
Adafruit_BMP280 bmp;    
bool bmp_ok = false;    
float alt_bmp = 0.0;

// =============================
// Trims
// =============================
Preferences prefs;  

struct TrimValues {
  float accel   = 128.0;
  float roll_lr = 128.0;
  float roll_fb = 128.0;
  float rudder  = 128.0;
  float sw      = 128.0;
} trims;

// =============================
// Variables de filtrado GPS
// =============================
double lat_window[4] = {0}, lon_window[4] = {0}, alt_window[4] = {0};
int win_idx = 0;
double lat_f = 0, lon_f = 0, alt_f = 0;
bool have_filter = false;
const double ALPHA = 0.35;

// =============================
// Filtrado
// =============================
double median4(double a, double b, double c, double d) {
  double arr[4] = {a,b,c,d};
  for(int i=0;i<3;i++) for(int j=i+1;j<4;j++) if(arr[j]<arr[i]) { double t=arr[i]; arr[i]=arr[j]; arr[j]=t; }
  return (arr[1]+arr[2])/2.0;
}

void filterGPS(double lat, double lon, double alt, double &lat_out, double &lon_out, double &alt_out) {
  lat_window[win_idx]=lat; lon_window[win_idx]=lon; alt_window[win_idx]=alt;
  win_idx=(win_idx+1)%4;
  double lat_m = median4(lat_window[0],lat_window[1],lat_window[2],lat_window[3]);
  double lon_m = median4(lon_window[0],lon_window[1],lon_window[2],lon_window[3]);
  double alt_m = median4(alt_window[0],alt_window[1],alt_window[2],alt_window[3]);
  if(!have_filter) { lat_f=lat_m; lon_f=lon_m; alt_f=alt_m; have_filter=true; }
  else { lat_f = ALPHA*lat_m + (1.0-ALPHA)*lat_f; lon_f = ALPHA*lon_m + (1.0-ALPHA)*lon_f; alt_f = ALPHA*alt_m + (1.0-ALPHA)*alt_f; }
  lat_out=lat_f; lon_out=lon_f; alt_out=alt_f;
}

// =============================
// Timestamp UNIX
// =============================
unsigned long toUnixTime(int y,int m,int d,int h,int min,int s){
  if(m<=2){y-=1; m+=12;}
  long a=y/100, b=2-a+a/4;
  long days=(long)(365.25*(y+4716))+(long)(30.6001*(m+1))+d+b-1524.5;
  unsigned long ts=(days-2440588)*86400UL + h*3600UL + min*60UL + s;
  return ts;
}

// =============================
// Trims
// =============================
void loadTrims(){
  prefs.begin("drone",true);
  trims.accel   = prefs.getFloat("accel",128.0);
  trims.roll_lr = prefs.getFloat("roll_lr",128.0);
  trims.roll_fb = prefs.getFloat("roll_fb",128.0);
  trims.rudder  = prefs.getFloat("rudder",128.0);
  trims.sw      = prefs.getFloat("switch",128.0);
  prefs.end();
}

void saveTrims(){
  prefs.begin("drone",false);
  prefs.putFloat("accel",trims.accel);
  prefs.putFloat("roll_lr",trims.roll_lr);
  prefs.putFloat("roll_fb",trims.roll_fb);
  prefs.putFloat("rudder",trims.rudder);
  prefs.putFloat("switch",trims.sw);
  prefs.end();
}

// =============================
// Envíos LoRa
// =============================
void sendTelemetry(double lat,double lon,double alt,double speed,double heading,unsigned long ts){
  StaticJsonDocument<256> doc;
  doc["t"]="TELEMETRY";
  JsonObject d = doc.createNestedObject("d");
  d["lat"]=lat; d["lon"]=lon; d["alt"]=alt;
  d["speed"]=speed; d["heading"]=heading;
  d["battery"]=0.0; d["status"]=0.0; d["current_wp"]=0.0; d["total_wp"]=0.0;
  doc["ts"]=ts;
  String payload; serializeJson(doc,payload);
  String msg="GS#"+payload+"#END";
  LoRa.beginPacket(); LoRa.print(msg); LoRa.endPacket();
  Serial.println(msg);
}

void sendTrims(){
  StaticJsonDocument<256> doc;
  doc["t"]="TRIM_DATA";
  JsonObject d=doc.createNestedObject("d");
  d["accel"]=trims.accel; d["roll_lr"]=trims.roll_lr;
  d["roll_fb"]=trims.roll_fb; d["rudder"]=trims.rudder; d["switch"]=trims.sw;
  doc["ts"]=millis();
  String payload; serializeJson(doc,payload);
  String msg="GS#"+payload+"#END";
  LoRa.beginPacket(); LoRa.print(msg); LoRa.endPacket();
  Serial.println("Enviando trims: "+msg);
}

void sendTrimAck(){
  StaticJsonDocument<128> doc;
  doc["t"]="TRIM_ACK"; doc["ts"]=millis();
  String payload; serializeJson(doc,payload);
  String msg="GS#"+payload+"#END";
  LoRa.beginPacket(); LoRa.print(msg); LoRa.endPacket();
}

// Eventos FIRE/PERSON
void sendEventToGS(const String &topic,double lat,double lon,double alt,unsigned long ts){
  StaticJsonDocument<256> doc;
  doc["t"]=topic;
  JsonObject d=doc.createNestedObject("d");
  d["lat"]=lat; d["lon"]=lon; d["alt"]=alt;
  doc["ts"]=ts;
  String payload; serializeJson(doc,payload);
  String msg="GS#"+payload+"#END";
  LoRa.beginPacket(); LoRa.print(msg); LoRa.endPacket();
  Serial.println("Evento enviado: "+msg);
}

// =============================
// Procesar JSON entrante
// =============================
void processIncomingJSON(const String &jsonIn,bool fromGS){
  StaticJsonDocument<256> doc;
  DeserializationError err=deserializeJson(doc,jsonIn);
  if(err){ Serial.println("Error parseando JSON entrante"); return; }
  const char* t=doc["t"]; if(!t) return;

  if(fromGS){
    if(strcmp(t,"GET_TRIMS")==0) sendTrims();
    else if(strcmp(t,"TRIM")==0){
      JsonObject d=doc["d"];
      if(!d.isNull()){
        trims.accel   = d["accel"]   | trims.accel;
        trims.roll_lr = d["roll_lr"] | trims.roll_lr;
        trims.roll_fb = d["roll_fb"] | trims.roll_fb;
        trims.rudder  = d["rudder"]  | trims.rudder;
        trims.sw      = d["switch"]  | trims.sw;
        saveTrims();
        sendTrimAck();
      }
    }
    return;
  }

  // Mensajes Raspberry
  if(strcmp(t,"FIRE")==0 || strcmp(t,"PERSON")==0){
    unsigned long ts=0;
    if(gps.date.isValid() && gps.time.isValid())
      ts=toUnixTime(gps.date.year(),gps.date.month(),gps.date.day(),
                    gps.time.hour(),gps.time.minute(),gps.time.second());
    sendEventToGS(t,lat_f,lon_f,alt_f,ts);
  } else {
    Serial.println("Evento desconocido desde Raspberry: "+String(t));
  }
}

// =============================
// Setup
// =============================
void setup(){
  Serial.begin(115200);
  SerialGPS.begin(9600,SERIAL_8N1,GPS_RX,GPS_TX);
  SerialRPI.begin(115200,RPI_RX,RPI_TX);

  LoRa.setPins(LORA_CS,LORA_RST,LORA_IRQ);
  if(!LoRa.begin(LORA_BAND)){
    Serial.println("{\"error\":\"Fallo inicialización LoRa\"}");
    while(true) delay(1000);
  }
  Serial.println("{\"status\":\"LoRa inicializado correctamente\"}");

  if(bmp.begin(0x76)){
    bmp_ok=true; bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,Adafruit_BMP280::SAMPLING_X2,
                                  Adafruit_BMP280::SAMPLING_X16,Adafruit_BMP280::FILTER_X16,
                                  Adafruit_BMP280::STANDBY_MS_63);
    Serial.println("BMP280 OK");
  } else Serial.println("No se detectó BMP280");

  loadTrims();
}

// =============================
// Loop
// =============================
void loop(){
  // GPS
  while(SerialGPS.available()>0){
    gps.encode(SerialGPS.read());
    if(gps.location.isUpdated()){
      double lat_raw=gps.location.lat();
      double lon_raw=gps.location.lng();
      double alt=bmp_ok?bmp.readAltitude(1013.25):gps.altitude.meters();
      double lat,lon,alt_filt;
      filterGPS(lat_raw,lon_raw,alt,lat,lon,alt_filt);
      unsigned long ts=0;
      if(gps.date.isValid() && gps.time.isValid())
        ts=toUnixTime(gps.date.year(),gps.date.month(),gps.date.day(),
                      gps.time.hour(),gps.time.minute(),gps.time.second());
      sendTelemetry(lat,lon,alt_filt,gps.speed.kmph(),gps.course.deg(),ts);
    }
  }

  // Serial Raspberry
  while(SerialRPI.available()){
    String jsonIn=SerialRPI.readStringUntil('\n');
    processIncomingJSON(jsonIn,false);
  }

  // LoRa (GS)
  int packetSize=LoRa.parsePacket();
  while(packetSize--){
    String jsonIn="";
    while(LoRa.available()) jsonIn+=(char)LoRa.read();
    processIncomingJSON(jsonIn,true);
  }
}
