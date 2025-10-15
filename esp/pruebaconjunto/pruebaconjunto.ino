/*
  ESP32 + u-blox NEO-7N
  Rastrillaje cuadriculado dentro de polígono (4 puntos)
  Salida PWM (un canal por pin) simulando receptor RC
  usando ledcWrite (no bloqueante) para cada canal.

  *** SEGURIDAD ***
  - PRUEBA EN BANCO SIN HÉLICES. DESARMADO. USE LIMITADORES DE SALIDA.
  - Ajuste modos del FC (ALT HOLD / POS HOLD) según corresponda.
  - La altura con GPS es poco fiable; ideal: barómetro o FC en Altitude Hold.

  Autor: ChatGPT
*/

#include <Arduino.h>
#include <TinyGPSPlus.h>
#include <HardwareSerial.h>
#include <math.h>

/********************** CONFIGURACIÓN GENERAL ************************/ 
// --- GPS ---
HardwareSerial gpsSerial(2);       // UART2
static const int GPS_RX = 16;      // RX del ESP32 (a TX del GPS)
static const int GPS_TX = 17;      // TX del ESP32 (normalmente sin uso)
static const uint32_t GPS_BAUD = 9600;
TinyGPSPlus gps;

// --- PWM ---
// Pines PWM (un canal por pin)
#define PWM_CH1 25 // Roll
#define PWM_CH2 26 // Pitch
#define PWM_CH3 27 // Throttle
#define PWM_CH4 14 // Yaw
#define PWM_CH5 32 // AUX1 (ARM)
#define PWM_CH6 33 // AUX2

const int pwmPins[] = {PWM_CH1, PWM_CH2, PWM_CH3, PWM_CH4, PWM_CH5, PWM_CH6};
const int pwmChannels[] = {0,1,2,3,4,5};
const int pwmFreq = 50;       // 50 Hz PWM RC estándar
const int pwmResolution = 16; // 16 bits resolución

static const uint16_t PWM_CH_MIN = 1000;
static const uint16_t PWM_CH_MID = 1500;
static const uint16_t PWM_CH_MAX = 2000;

// --- Control / navegación ---
static const double WAYPOINT_RADIUS_M = 5.0;
static const double GRID_SPACING_M    = 20.0;
static const double CRUISE_PITCH_CMD  = 1600;
static const double BASE_THROTTLE     = 1500;
static const double DESIRED_ALT_M     = 20.0;

static const double K_YAW  = 2.5;
static const double K_PITCH_DIST = 5.0;
static const double K_ALT  = 6.0;

static const bool START_ARMED = false;

/********************** POLÍGONO (REEMPLAZAR) ************************/
struct Coord { double lat; double lon; };
Coord poly[4] = {
  { -34.603700, -58.381600 },
  { -34.603700, -58.380000 },
  { -34.602000, -58.380000 },
  { -34.602000, -58.381600 }
};
static const int POLY_N = 4;

/********************** UTILS GEO ************************/
static const double EARTH_R = 6371000.0; // m

inline double rad(double d){ return d * PI / 180.0; }
inline double deg(double r){ return r * 180.0 / PI; }

double haversine(double lat1, double lon1, double lat2, double lon2){
  double dLat = rad(lat2-lat1), dLon = rad(lon2-lon1);
  double a = sin(dLat/2)*sin(dLat/2) + cos(rad(lat1))*cos(rad(lat2))*sin(dLon/2)*sin(dLon/2);
  double c = 2*atan2(sqrt(a), sqrt(1-a));
  return EARTH_R * c;
}

double bearingTo(double lat1,double lon1,double lat2,double lon2){
  double y = sin(rad(lon2-lon1)) * cos(rad(lat2));
  double x = cos(rad(lat1))*sin(rad(lat2)) - sin(rad(lat1))*cos(rad(lat2))*cos(rad(lon2-lon1));
  double brng = atan2(y,x);
  brng = fmod(deg(brng)+360.0,360.0);
  return brng;
}

bool pointInPoly(double lat,double lon,const Coord* P,int n){
  bool inside=false; int j=n-1;
  for(int i=0;i<n;i++){
    bool cond = ((P[i].lon>lon)!=(P[j].lon>lon)) &&
                (lat < (P[j].lat-P[i].lat)*(lon-P[i].lon)/(P[j].lon-P[i].lon)+P[i].lat);
    if(cond) inside=!inside; j=i;
  }
  return inside;
}

void metersToDeg(double lat0, double dx_m, double dy_m, double &dLat, double &dLon){
  dLat = dy_m / 111320.0;
  double mPerDegLon = 111320.0 * cos(rad(lat0));
  dLon = dx_m / mPerDegLon;
}

/********************** GENERADOR DE GRID *****************/
Coord lerp(const Coord&a,const Coord&b,double t){ return { a.lat + (b.lat-a.lat)*t, a.lon + (b.lon-a.lon)*t }; }

const int MAX_WP = 400;
Coord waypoints[MAX_WP];
int wpCount=0;

void buildGridCoverage(double spacing_m){
  wpCount=0;
  Coord A=poly[0], B=poly[1], C=poly[2], D=poly[3];
  double stepLat, stepLon;
  metersToDeg((A.lat+D.lat)/2.0, 0, spacing_m, stepLat, stepLon);
  double lenLatDeg = fabs(D.lat - A.lat);
  int stripes = max(1, (int)floor(lenLatDeg / fabs(stepLat)));

  for(int s=0; s<=stripes; ++s){
    double t = stripes==0 ? 0.0 : (double)s/(double)stripes;
    Coord L = lerp(A,D,t);
    Coord R = lerp(B,C,t);
    const int samples = 20;
    for(int i=0;i<=samples;i++){
      double u = (s%2==0) ? (double)i/samples : 1.0 - (double)i/samples;
      Coord P = lerp(L,R,u);
      if(pointInPoly(P.lat,P.lon,poly,POLY_N)){
        if(wpCount<MAX_WP) waypoints[wpCount++] = P;
      }
    }
  }
}

/********************** PWM: envío no bloqueante *******************/
uint16_t chUs[6] = {PWM_CH_MID,PWM_CH_MID,PWM_CH_MID,PWM_CH_MID,1000,1000};

void pwmInit(){
  for(int i=0;i<6;i++){
    ledcSetup(pwmChannels[i], pwmFreq, pwmResolution);
    ledcAttachPin(pwmPins[i], pwmChannels[i]);
  }
}

void pwmWrite(){
  for(int i=0;i<6;i++){
    // duty = us / (1000000/freq)
    uint32_t duty = (uint32_t)((chUs[i] * pow(2,pwmResolution) * pwmFreq) / 1000000UL);
    ledcWrite(pwmChannels[i], duty);
  }
}

/********************** CONTROL SIMPLE ******************************/
uint16_t mapCmd(double val, double lim){
  val = constrain(val, -lim, lim);
  double t = (val + lim) / (2.0*lim);
  return (uint16_t)(PWM_CH_MIN + t*(PWM_CH_MAX-PWM_CH_MIN));
}

void setDisarmed(){
  chUs[0]=PWM_CH_MID;
  chUs[1]=PWM_CH_MID;
  chUs[2]=1000;
  chUs[3]=PWM_CH_MID;
  chUs[4]=1000;
  chUs[5]=1000;
}

void setArmed(bool on){ chUs[4] = on ? 2000 : 1000; }

/********************** ESTADO NAVEGACIÓN ****************************/
int wpIndex = 0;
bool gridBuilt = false;
bool armed = START_ARMED;

void setup(){
  Serial.begin(115200);
  delay(100);
  Serial.println("Init ESP32 GPS + PWM");
  gpsSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX, GPS_TX);
  pwmInit();
  setDisarmed();
  setArmed(armed);
}

void computeRCFromNav(double lat,double lon,double alt){
  if(!gridBuilt){
    buildGridCoverage(GRID_SPACING_M);
    wpIndex = 0;
    gridBuilt = true;
    Serial.printf("WP generados: %d\n", wpCount);
  }
  if(wpIndex >= wpCount){
    chUs[0]=PWM_CH_MID;
    chUs[1]=PWM_CH_MID;
    chUs[3]=PWM_CH_MID;
    chUs[2]=BASE_THROTTLE;
    return;
  }

  Coord tgt = waypoints[wpIndex];
  double dist = haversine(lat,lon,tgt.lat,tgt.lon);
  double brg  = bearingTo(lat,lon,tgt.lat,tgt.lon);
  double hdgErr = brg;
  if(hdgErr>180) hdgErr -= 360.0;

  double yawCmd = PWM_CH_MID + K_YAW * hdgErr;
  yawCmd = constrain(yawCmd, PWM_CH_MIN, PWM_CH_MAX);

  double pitchCmd = PWM_CH_MID + min(300.0, K_PITCH_DIST * dist);
  pitchCmd = constrain(pitchCmd, PWM_CH_MIN, PWM_CH_MAX);

  double rollCmd = PWM_CH_MID;

  double thr = BASE_THROTTLE;
  if(gps.altitude.isValid()){
    double altErr = DESIRED_ALT_M - gps.altitude.meters();
    thr = BASE_THROTTLE + K_ALT * altErr;
  }
  thr = constrain(thr, PWM_CH_MIN, PWM_CH_MAX);

  chUs[0] = (uint16_t)rollCmd;
  chUs[1] = (uint16_t)pitchCmd;
  chUs[2] = (uint16_t)thr;
  chUs[3] = (uint16_t)yawCmd;

  if(dist < WAYPOINT_RADIUS_M){
    wpIndex++;
    Serial.printf("Siguiente WP: %d / %d\n", wpIndex, wpCount);
  }
}

void loop(){
  while(gpsSerial.available()>0){ gps.encode(gpsSerial.read()); }

  if(gps.location.isUpdated()){
    double lat = gps.location.lat();
    double lon = gps.location.lng();
    computeRCFromNav(lat, lon, gps.altitude.meters());
    Serial.printf("Lat: %.6f Lon: %.6f Alt: %.1f  WP:%d/%d\n", lat, lon, gps.altitude.isValid()?gps.altitude.meters():NAN, wpIndex, wpCount);
  }

  pwmWrite();
}

