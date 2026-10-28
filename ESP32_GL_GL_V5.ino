//////////////////////////////////////////////////////////////////////////////////
//
// ESP32 Garden & Garage Light Controller V5
//
//  11th September 2026
//  17th September - Split internal garage lights across Q2 & Q4, 
//                   implemented magnetic front & rear garage door switches.
//  3rd October  -  simplified alexa dimming and eeprom saving. implemented 
//  9th October  - added reboot every Sunday@4am
/////////////////////////////////////////////////////////////////////////////////

#include <WiFi.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <fauxmoESP.h>
#include <time.h> 
#include <math.h> 
#include <EEPROM.h>
#
bool cpu = false;
unsigned long start;
#define EEPROM_SIZE 512
// A unique value used to determine whether EEPROM has been initialised
#define EEPROM_MAGIC 0x45A3B7C9
// Increment this if you change the structure/layout
#define EEPROM_VERSION 1

struct    {
  bool frontDoorSensor = false;
  bool rearDoorSensor = false;
  bool ntp = false;
}status;




struct Settings
{
    int  firmware;
    int  dimValue;
    bool enabled;
    char deviceName[32];
};
Settings settings;

struct EEPROMHeader
{
    uint32_t magic;
    uint16_t version;
};

// Door Sensors
#define GARAGEDOORSENSOR_FRONT 35
#define GARAGEDOORSENSOR_REAR 34

// Alexa Integration
fauxmoESP alexa;
unsigned long lastFauxmoRestart = 0;
const unsigned long RESTART_INTERVAL = 900000; // Restart every 15 minutes


// Location: Warwick, Warwickshire, United Kingdom
// --- Warwick, UK Location Data ---
const float WARWICK_LAT = 52.2815;  
const float WARWICK_LON = -1.5816;  

unsigned long wifiTimeout = 30 * 1000;        // 30s
unsigned long connectionStartTime;
unsigned long flashLed = 2000;

// UK timezone with automatic BST
#define TZ_INFO "GMT0BST,M3.5.0/1,M10.5.0/2"
#define MIDNIGHT 1380      // 23 * 60 = 2300hrs
struct tm timeinfo;
int timeInMinutes= 0;
int dusk=0;
int dawn=0;

// --- Configuration Constants ---
const char* ssid     = "espdevnet";
const char* password = "ac6e882db2";
const char* HOSTNAME  = "ESP32_GL_GL_AI_V5";
const char* SYSLOG_HOST =   "192.168.1.235"; 
const int SYSLOG_PORT     = 514;
const int UDP_PORT        = 9095;

// --- Pin Assignments ---
const int PIN_GARAGE_LIGHT = 21;
const int PIN_GARAGE_LIGHT_FRONT = 23;
const int PIN_INDICATOR = 2;
const int PIN_GARDEN_LIGHT = 19;
const int PIN_ONE_WIRE_BUS = 13; 

// --- PWM Constants ---
const int PWM_CHANNEL    = 0;
const int PWM_FREQ       = 5000;
const int PWM_RESOLUTION = 8;
int PWM_DUTY_80    = 204;       // 80% of 255 discrete steps, 70% = 178

// --- Timing Constants ---
const unsigned long GARAGE_TIMEOUT_MS  =  10 * 60  * 1000;      //  10 mins // garage light, on-time
const unsigned long TELEMETRY_INTERVAL =  60 * 60  * 1000;      //  60 mins // telemetry
const unsigned long GARDEN_LOG_INTERVAL = 120 * 60 * 1000;     //  120 mins // log status of garden lights


// --- Device Objects ---
WiFiUDP udpListener;
WiFiUDP syslogUdp;
WiFiClient client;
OneWire oneWire(PIN_ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);


// --- State Variables ---
bool garageLightState = false;
bool gardenLightManualOverride = false;
bool gardenLightManualState = false;
bool gardenLightState = false;

bool alexaOverride_GardenLights = false;
bool alexaOverride_GarageLights = false;

unsigned long gardenTurnOnTime = 0; 
unsigned long garageTurnOnTime = 0;
unsigned long lastTelemetryTime = 0;
unsigned long lastGardenLogTime = 0;
float currentTemperature = 0.0;

// --- Function Prototypes ---
bool initWiFi();
void initOTA();
void initAlexa();
void handleUdpPackets();
void handleGarageTimer();
void handleGardenLightAutomation();
void sendTelemetryLog();
String getUptimeString();

void setup() {
    Serial.begin(115200);
      
      pinMode(2,OUTPUT);
      digitalWrite(2,HIGH);
      
      if(!initWiFi() ) {
        Serial.printf("Could not connect to Network!\n");
        
        WiFi.disconnect();
        delay(10000);
        ESP.restart();
      }
              initOTA();
              initialiseEEPROM();
                  //sntp_set_time_sync_notification_cb(timeSyncCallback);               
   
    // Configure Outputs
    pinMode(PIN_GARAGE_LIGHT, OUTPUT);
    digitalWrite(PIN_GARAGE_LIGHT, LOW);
    pinMode(PIN_GARAGE_LIGHT_FRONT, OUTPUT);
    digitalWrite(PIN_GARAGE_LIGHT_FRONT, LOW);
    
    pinMode(GARAGEDOORSENSOR_FRONT,INPUT);
    pinMode(GARAGEDOORSENSOR_REAR,INPUT);
    
    // Set up ESP32 LEDC PWM Hardware Engine
    ledcAttach(PIN_GARDEN_LIGHT, PWM_FREQ, PWM_RESOLUTION);
    ledcWrite(PIN_GARDEN_LIGHT, 0);


  
    // Initialise
    
    initAlexa();
    sensors.begin();
    udpListener.begin(UDP_PORT);
    ledTest();
    isNightTime();

    // readEEPROM
if(!readEEPROM(settings) )
{
      logToSyslog(3,"Error Reading EEPROM" );
      settings.dimValue = 203;                     // set a default and continue

}   
  logToSyslog(6,"ESP32 Garden & Garage Light Control, Started Successfully");

  Serial.printf("Time: %d:%d \n",timeinfo.tm_hour, timeinfo.tm_min );
    Serial.printf("ESP32 Garden & Garage Light Control, Running.   \n");
      start = millis();  
 
}

void loop() {
 
    checkRestart();
    
    if((millis() - start) > flashLed ){
          digitalWrite(2,cpu);
          start=millis(); 
          cpu = !cpu;   
    }
    
    alexa.handle();
 // Periodic restart to flush memory and reset connections
    if (millis() - lastFauxmoRestart > RESTART_INTERVAL) {
        lastFauxmoRestart = millis();
        alexa.enable(false);
        delay(100);
        alexa.enable(true);
        if(WiFi.status() != WL_CONNECTED) {
              WiFi.reconnect();
        }
    }


    isNightTime();
    ArduinoOTA.handle();  
    handleUdpPackets();
    handleGarageTimer();
    handleGardenLightAutomation();  
    getLocalTime(&timeinfo);
    
      // front and rear magnetic door garage switches
      status.frontDoorSensor = digitalRead(GARAGEDOORSENSOR_FRONT);
      status.rearDoorSensor = digitalRead(GARAGEDOORSENSOR_REAR);
      

 if( (!digitalRead(GARAGEDOORSENSOR_FRONT) || !digitalRead(GARAGEDOORSENSOR_REAR) ) )      // sensor(s) are ACTIVE LOW, AND extendedlamptimer is NOT active
  {             
               if(alexaOverride_GarageLights){
                                return;
                }
              //logToSyslog(6,"Garage Lights are ON!");
              garageLightState = true;
              digitalWrite(PIN_GARAGE_LIGHT, HIGH);
              digitalWrite(PIN_GARAGE_LIGHT_FRONT, HIGH);   
              garageTurnOnTime = millis();

  }
      
    // Asynchronous Periodic  Telemetry Engine
    if (millis() - lastTelemetryTime >= TELEMETRY_INTERVAL) {
        sendTelemetryLog();
        lastTelemetryTime = millis();
    }
    delay(50);
}

// LED TEST

void ledTest()  {
Serial.printf("Starting LED Test \n");
digitalWrite(PIN_GARAGE_LIGHT, HIGH);
digitalWrite(PIN_GARAGE_LIGHT_FRONT, HIGH);
for(int x = 0;x < 255; ++x )
{
  ledcWrite(PIN_GARDEN_LIGHT, x );
        delay(15);
  
}
for(int x = 255;x > 0; x-- )
{
  ledcWrite(PIN_GARDEN_LIGHT, x );
        delay(15);
  
}

ledcWrite(PIN_GARDEN_LIGHT, 0 );
digitalWrite(PIN_GARAGE_LIGHT, LOW); 
digitalWrite(PIN_GARAGE_LIGHT_FRONT, LOW);

}
void checkRestart()  {
  if(timeinfo.tm_wday == 0 && timeinfo.tm_hour == 4 && timeinfo.tm_min == 0 && timeinfo.tm_sec == 0 )
  {
    logToSyslog(2,"ESP32 scheduled Reboot!");
    delay(2000);
    ESP.restart();
  }
}

// WIFI
bool initWiFi() {
  
    WiFi.mode(WIFI_STA); 
    WiFi.setSleep(false);
    WiFi.begin(ssid, password);
    Serial.print("Connecting to WiFi..");
    connectionStartTime = millis();
    
    while (WiFi.status() != WL_CONNECTED && (millis() - connectionStartTime) < wifiTimeout ) {
            delay(500);
              Serial.printf(".");
        
    }

    if(WiFi.status() == WL_CONNECTED)
    {
        Serial.printf("\nWiFi Connected. IP = %s",WiFi.localIP().toString().c_str());
        Serial.println("\nStarting NTP Client.");
        // NTP
          configTzTime( TZ_INFO, "uk.pool.ntp.org","pool.ntp.org","time.nist.gov" );
          getLocalTime(&timeinfo);
      return true;
    }

    return false;
 }

 

void initOTA() {
    ArduinoOTA.setHostname(HOSTNAME);
    ArduinoOTA.begin();
}

void initAlexa() {
    alexa.createServer(true);
    alexa.setPort(80);
    alexa.enable(true);
    alexa.addDevice("Garage Lights");
    alexa.addDevice("Garden Lights");
     
    alexa.onSetState([](unsigned char device_id, const char * device_name, bool state, unsigned char value) 
    {
          
        
        if (strcmp(device_name, "Garage Lights") == 0) {
            garageLightState = state;
            alexaOverride_GarageLights = state;

           
           //DEBUG logToSyslog(7,device_id);
            
            digitalWrite(PIN_GARAGE_LIGHT, garageLightState ? HIGH : LOW);
            digitalWrite(PIN_GARAGE_LIGHT_FRONT, garageLightState ? HIGH : LOW);
            if (garageLightState) {
                garageTurnOnTime = millis();
               
                logToSyslog(6,"Internal Garage Lights turned ON via Alexa");
            } else {
               logToSyslog(6,"Internal Garage Lights turned OFF via Alexa");
            }
        }

if (strcmp(device_name, "Garden Lights") == 0) {
            gardenLightState = state;
             alexaOverride_GardenLights = state;
            // getEEPROM_DIM_VALUE

            Serial.printf("Garden Lights Alexa Value: %d Settings Dim Value: %d \n",value,settings.dimValue );
            if(value != settings.dimValue)
            {
                settings.dimValue = value;
                updateEEPROM(settings);
            }

          
            
            if(gardenLightState) {
                ledcWrite(PIN_GARDEN_LIGHT,settings.dimValue);       // settings.dimValue = dim factor;
                
            }
            else
            {
              ledcWrite(PIN_GARDEN_LIGHT, 0);

              Serial.printf("Garden Lights Off \n");
            }
            
            
            
            if (gardenLightState) {
                gardenTurnOnTime = millis();
                
                logToSyslog(6,"External Garden Lights turned ON via Alexa");
            } else {
               logToSyslog(6,"External Garden Lights turned OFF via Alexa");
            }
        }
    
    });
}


void handleUdpPackets() {
    int packetSize = udpListener.parsePacket();
    if (packetSize) {
        char packetBuffer[64];
        int len = udpListener.read(packetBuffer, sizeof(packetBuffer) - 1);
        if (len > 0) {
            packetBuffer[len] = '\0';
        }
        
        String command = String(packetBuffer);
        command.trim();

        if (command == "INTERNAL_GARAGE_LIGHT_ON") {
            garageLightState = true;
            digitalWrite(PIN_GARAGE_LIGHT, HIGH);
            digitalWrite(PIN_GARAGE_LIGHT_FRONT, HIGH);
            
            garageTurnOnTime = millis();
            logToSyslog(6,"Garage Lights turned ON via UDP");
        } 
        else if (command == "INTERNAL_GARAGE_LIGHT_OFF") {
            garageLightState = false;
            digitalWrite(PIN_GARAGE_LIGHT, LOW);
            digitalWrite(PIN_GARAGE_LIGHT_FRONT, LOW);
            logToSyslog(6,"Garage Lights turned OFF via UDP");
        } 
        else if (command == "EXTERNAL_GARDEN_LIGHT_ON") {
            gardenLightManualOverride = true;
            gardenLightManualState = true;
            ledcWrite(PIN_GARDEN_LIGHT, settings.dimValue);
           logToSyslog(6,"Garden Lights turned ON via UDP (Override)");
        } 
        else if (command == "EXTERNAL_GARDEN_LIGHT_OFF") {
            gardenLightManualOverride = false;
            gardenLightManualState = false;
            ledcWrite(PIN_GARDEN_LIGHT, 0);
           logToSyslog(6,"Garden Lights turned OFF via UDP (Override)");
        } 
        else if (command == "REBOOT") {
            logToSyslog(6,"Reboot command received via UDP. Restarting...");
            delay(1000);
            ESP.restart();
        }
    }
}

void handleGarageTimer() {
    
    if(alexaOverride_GarageLights){
      return;
    }
    
    
    if (!alexaOverride_GarageLights && garageLightState && (millis() - garageTurnOnTime >= GARAGE_TIMEOUT_MS)) {
        garageLightState = false;
        digitalWrite(PIN_GARAGE_LIGHT, LOW);
        digitalWrite(PIN_GARAGE_LIGHT_FRONT, LOW);
       logToSyslog(6,"Garage Light turned OFF automatically after 10 minutes");
    }
}

void handleGardenLightAutomation() {
   String k ="";
    
    if (gardenLightManualOverride) {
        return;
    }
if (alexaOverride_GardenLights) {
        return;
    }
    
  
   
     
   
  
    // Apply hardware driver output state 
    if (isNightTime() ) {
        ledcWrite(PIN_GARDEN_LIGHT, settings.dimValue);
         k = "[Automation] Garden Lights are ON";
    } else {
        ledcWrite(PIN_GARDEN_LIGHT, 0);
        k = "[Automation] Garden Lights are OFF";
    }

    // --- Asynchronous Gated Console & Syslog Engine ---
    if (millis() - lastGardenLogTime >= GARDEN_LOG_INTERVAL) {
        logToSyslog(6,k);
        lastGardenLogTime = millis();
    }
}


String getUptimeString() {
    unsigned long ms = millis();
    unsigned long totalSecs = ms / 1000;
    unsigned long hours = totalSecs / 3600;
    unsigned long mins = (totalSecs % 3600) / 60;
    unsigned long secs = totalSecs % 60;
    
    char buf[32];
    snprintf(buf, sizeof(buf), "%02lu:%02lu:%02lu", hours, mins, secs);
    return String(buf);
}

void sendTelemetryLog() {
    sensors.requestTemperatures();
    currentTemperature = sensors.getTempCByIndex(0);

    char telemetryMsg[256];
    snprintf(telemetryMsg, sizeof(telemetryMsg),
             "IP: %s |RSSI: %d dBm |BSSID: %s |UPTIME: %s |TEMP: %.2fC |SUNSET: %d |SUNRISE: %d |LIGHT DIM: %d |TIME NOW: %d | ",
             WiFi.localIP().toString().c_str(),
             WiFi.RSSI(),
             WiFi.BSSIDstr().c_str(),
             getUptimeString().c_str(),
             currentTemperature,
             dusk,
             dawn,
             settings.dimValue,
             timeInMinutes );
          
             
    
    logToSyslog(8,telemetryMsg);
}




// --- Dynamic Offline Sunrise/Sunset Calculator Engine ---
int calculateSolarMinutes(int year, int month, int day, bool isSunset) {


/**
 * Calculates Sunrise or Sunset time in minutes from midnight for Warwick, UK.
 * Includes automatic Daylight Saving Time (BST) adjustment.
 * 
 * @param year        The calendar year (e.g. 2026)
 * @param month       The month (1 to 12)
 * @param day         The day of the month (1 to 31)
 * @param isSunset    Set to true for SUNSET, false for SUNRISE
 * @return            Time in minutes from midnight (0 = midnight)
 */

    // 1. Hardcoded coordinates for Warwick, Warwickshire, England
    const double latitude = 52.2835; 
    const double longitude = -1.5892; // West is negative

    // 2. Standard Zeniths for Sunrise/Sunset calculations
    const double zenith = 90.83333333333333; 

    // 3. Calculate Day of the Year
    int N1 = floor(275 * month / 9);
    int N2 = floor((month + 9) / 12);
    int N3 = (1 + floor((year - 4 * floor(year / 4) + 2) / 3));
    int N = N1 - (N2 * N3) + day - 30;

    // 4. Convert longitude to hour value and estimate an approximate time
    double lngHour = longitude / 15.0;
    double t = N + ((isSunset ? 18.0 : 6.0) - lngHour) / 24.0;

    // 5. Calculate Sun's Mean Anomaly (M)
    double M = (0.9856 * t) - 3.289;

    // 6. Calculate Sun's True Longitude (L)
    double L = M + (1.916 * sin(M * DEG_TO_RAD)) + (0.020 * sin(2 * M * DEG_TO_RAD)) + 282.634;
    L = fmod(L + 360.0, 360.0);

    // 7. Calculate Sun's Right Ascension (RA)
    double RA = atan(0.91764 * tan(L * DEG_TO_RAD)) * RAD_TO_DEG;
    RA = fmod(RA + 360.0, 360.0);

    // Adjust Right Ascension quadrant
    double Lquadrant  = (floor(L / 90.0)) * 90.0;
    double RAquadrant = (floor(RA / 90.0)) * 90.0;
    RA = RA + (Lquadrant - RAquadrant);
    RA = RA / 15.0; // Convert to hours

    // 8. Calculate Sun's Declination (sinDec and cosDec)
    double sinDec = 0.39782 * sin(L * DEG_TO_RAD);
    double cosDec = cos(asin(sinDec));

    // 9. Calculate Local Hour Angle (H)
    double cosH = (cos(zenith * DEG_TO_RAD) - (sinDec * sin(latitude * DEG_TO_RAD))) / (cosDec * cos(latitude * DEG_TO_RAD));
    
    // Return early if there is no sunrise/sunset (polar regions - rare but good for safety)
    if (cosH > 1) return -1;  // Sun stays down
    if (cosH < -1) return -1; // Sun stays up

    // 10. Calculate local hour angle (H) in hours
    double H = isSunset ? acos(cosH) * RAD_TO_DEG : 360.0 - acos(cosH) * RAD_TO_DEG;
    H = H / 15.0;

    // 11. Calculate local Mean Time of event (T) in UTC hours
    double T = H + RA - (0.06571 * t) - 6.622;
    double UT = T - lngHour;
    UT = fmod(UT + 24.0, 24.0); // Final UTC Time in decimal hours

    // Convert to absolute UTC minutes from midnight
    int minutesFromMidnightUTC = round(UT * 60.0);

    // 12. Calculate British Summer Time (DST) rules dynamically 
    // UK DST begins last Sunday of March, ends last Sunday of October
    bool isDST = false;
    
    if (month > 3 && month < 10) {
        isDST = true;
    } else if (month == 3) {
        // Last Sunday in March helper calculation
        int lastSundayMarch = 31 - ((5 + (int)(year * 5 / 4)) % 7);
        if (day >= lastSundayMarch) isDST = true;
    } else if (month == 10) {
        // Last Sunday in October helper calculation
        int lastSundayOctober = 31 - ((2 + (int)(year * 5 / 4)) % 7);
        if (day < lastSundayOctober) isDST = true;
    }

    // Apply standard UK offset: GMT = UTC+0, BST = UTC+1 hour (+60 minutes)
    int localMinutesFromMidnight = minutesFromMidnightUTC + (isDST ? 60 : 0);
    
    // Ensure the edge calculation handles rollover wraps beautifully
    return fmod(localMinutesFromMidnight + 1440, 1440);
}
  
// --- Helper: Verify Night time Window ---
bool isNightTime() {

  dawn = calculateSolarMinutes(timeinfo.tm_year + 1900, timeinfo.tm_mon + 1, timeinfo.tm_mday, false);
  dusk = calculateSolarMinutes(timeinfo.tm_year + 1900, timeinfo.tm_mon + 1, timeinfo.tm_mday, true);
  timeInMinutes = timeinfo.tm_hour * 60 + timeinfo.tm_min;
  
  return (timeInMinutes >= dusk && timeInMinutes <= MIDNIGHT);

  
}


// --- Helper: Core Syslog Transmission Engine ---
void logToSyslog(uint8_t severity, const String& message) 
{String severityLevel[11] = {"EMERGENCY","ALERT","CRITICAL","ERROR","WARNING","NOTIFICATION","INFO","DEBUG","TELEMETRY","SUCCESS","NO ENTRY" };
  if (WiFi.status() != WL_CONNECTED) {
    return;
  }
  uint8_t pri = severity; 
  syslogUdp.beginPacket(SYSLOG_HOST, SYSLOG_PORT);
  syslogUdp.printf("[ %s ] %s: %s", severityLevel[pri], HOSTNAME, message.c_str());
  syslogUdp.endPacket();
}

void maintainWiFi() {
     
   
  if (WiFi.status() != WL_CONNECTED) {
    //Serial.println("[WiFi Monitor] Connection lost. Attempting reconnection...");
    WiFi.reconnect();
  }
}

bool initialiseEEPROM()
{
    if (!EEPROM.begin(EEPROM_SIZE))
    {
        Serial.println("ERROR: EEPROM.begin() failed");
        return false;
    }

    EEPROMHeader header;

    EEPROM.get(0, header);

    // Check whether EEPROM contains valid initialisation information
    if (header.magic == EEPROM_MAGIC &&
        header.version == EEPROM_VERSION)
    {
        // EEPROM is already initialised
        EEPROM.get(sizeof(EEPROMHeader), settings);

        Serial.println("EEPROM already initialised.");
        return true;
    }

    // -------------------------------------------------------------------------
    // EEPROM is not initialised
    // -------------------------------------------------------------------------

    Serial.println("EEPROM not initialised.");
    Serial.println("Writing default settings...");

    setDefaultSettings();

    header.magic = EEPROM_MAGIC;
    header.version = EEPROM_VERSION;

    // Write header
    EEPROM.put(0, header);

    // Write structure immediately after header
    EEPROM.put(sizeof(EEPROMHeader), settings);

    // ESP32 EEPROM requires commit()
    if (!EEPROM.commit())
    {
        Serial.println("ERROR: EEPROM.commit() failed");
        return false;
    }

    Serial.println("EEPROM initialised.");

    return false;
}


// -----------------------------------------------------------------------------
// Read settings from EEPROM
//
// Returns:
//   true  = settings successfully read
//   false = EEPROM was not initialised
// -----------------------------------------------------------------------------

bool readEEPROM(Settings &data)
{
    EEPROMHeader header;

    EEPROM.get(0, header);

    // Check EEPROM validity
    if (header.magic != EEPROM_MAGIC ||
        header.version != EEPROM_VERSION)
    {
        Serial.println("EEPROM is not initialised.");
        return false;
    }

    // Read structure from EEPROM
    EEPROM.get(sizeof(EEPROMHeader), data);

    return true;
}


// -----------------------------------------------------------------------------
// Update EEPROM from a structure
//
// Pass the structure you want stored.
//
// Returns:
//   true  = successfully written
//   false = EEPROM write/commit failed
// -----------------------------------------------------------------------------

bool updateEEPROM(const Settings &data)
{
    // Make sure the EEPROM has been started
    // This is safe to call after initialiseEEPROM().
    EEPROM.put(sizeof(EEPROMHeader), data);

    if (!EEPROM.commit())
    {
        Serial.println("ERROR: EEPROM.commit() failed");
        return false;
    }

    // Also update our working copy
    settings = data;

    Serial.println("EEPROM updated.");

    return true;
}
void setDefaultSettings()
{
    settings.firmware = 5;
    settings.dimValue = 203;
    settings.enabled = true;

    strncpy(settings.deviceName, HOSTNAME, sizeof(settings.deviceName));

    // Make absolutely sure the string is terminated
    settings.deviceName[sizeof(settings.deviceName) - 1] = '\0';
}
