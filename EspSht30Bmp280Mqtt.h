#ifndef ESPSHT30BMP280MQTT_ESPSHT30BMP280MQTT_H
#define ESPSHT30BMP280MQTT_ESPSHT30BMP280MQTT_H


#include "Network.h"         // Networking code.
#include "privateInfo.h"     // Location of Wi-Fi and MQTT settings.
#include <Adafruit_BMP280.h> // The Adafruit library for BMP280 sensor.
#include <Adafruit_SHT31.h>  // Driver library for the SHT30.  This library includes Wire.h.
#include <ArduinoJson.h>     // ArduinoJson by Benoît Blanchon: https://arduinojson.org/


// Commented-out variables are included in privateInfo.h, which is not added to GitHub.
//const char *wifiSsid = "nunya";     // Wi-Fi SSID.  Defined in privateInfo.h
//const char *wifiPassword = "nunya"; // Wi-Fi password.  Defined in privateInfo.h
//const char *mqttBroker = "nunya";   // The broker address.  Defined in privateInfo.h
//const unsigned int mqttPort = 1883; // The broker port.  Defined in privateInfo.h
//const char *otaPass = "nunya";      // OTA password.


char ipAddress[16];                                     // A character array to hold the IP address.
char macAddress[18];                                    // A character array to hold the MAC address, and append a dash and 3 numbers.
long rssi;                                              // A global to hold the Received Signal Strength Indicator.
unsigned int printInterval = 7000;                      // How long to wait between telemetry printouts.
unsigned int publishInterval = 20000;                   // How long to wait between telemetry publishes.
unsigned int wifiConnectCount = 0;                      // A counter for how many times the wifiConnect() function has been called.
unsigned int mqttConnectCount = 0;                      // A counter for how many times the mqttConnect() function has been called.
unsigned int invalidValueCount = 0;                     // A counter of how many times invalid values have been measured.
unsigned int publishNow = 0;                            // A flag to indicate that a publish should happen immediately.
unsigned long printCount = 0;                           // A counter of how many times the stats have been printed.
unsigned long publishCount = 0;                         // A counter of how many times the stats have been published.
unsigned long callbackCount = 0;                        // The number of times a callback was received.
unsigned long lastPrintTime = 0;                        // The last time telemetry was printed.
unsigned long lastPublishTime = 0;                      // The last time a MQTT publish was performed.
unsigned long lastWifiConnectTime = 0;                  // The last time a Wi-Fi connection was attempted.
unsigned long lastMqttConnectionTime = 0;               // The last time a MQTT broker connection was attempted.
unsigned long wifiCoolDownInterval = 10000;             // How long to wait between Wi-Fi connection attempts.
unsigned long mqttCoolDownInterval = 10000;             // How long to wait between MQTT broker connection attempts.
unsigned long wifiConnectionTimeout = 15000;            // The amount of time to wait for a Wi-Fi connection.
unsigned long ledBlinkInterval = 200;                   // The interval between telemetry processing times.
unsigned long lastLedBlinkTime = 0;                     // The time of the last telemetry process.
const unsigned int ONBOARD_LED = 2;                     // The GPIO which the onboard LED is connected to.
const unsigned int JSON_DOC_SIZE = 512;                 // The ArduinoJson document size.
const unsigned int MILLIS_IN_SEC = 1000;                // The number of milliseconds in one second.
const char *hostName = "OutdoorESP8266";                // The OTA hostname.
float seaLevelPressure = 1026.1;                        // The local sea-level pressure. Provo Airport: https://forecast.weather.gov/data/obhistory/KPVU.html
float sht30TempCArray[] = { -21.12, 21.12, 88.88 };     // An array to hold the 3 most recent Celsius values.
float sht30HumidityArray[] = { 1.1, 21.12, 55.55 };     // An array to hold the 3 most recent values.
float bmpTempCArray[] = { -21.12, 21.12, 88.88 };       // An array to hold the 3 most recent Celsius values, initialized to reasonable levels.
float bmpPressureHPaArray[] = { 8.7, 882.64, 1083.8 };  // An array to hold the 3 most recent barometric pressure values, initialized to reasonable levels.
float bmpAltitudeMArray[] = { -413.0, 1337.0, 3108.0 }; // An array to hold the 3 most recent barometric pressure values, initialized to reasonable levels.


Adafruit_SHT31 sht30 = Adafruit_SHT31();
Adafruit_BMP280 bmp280;


void setupSht30();
void setupBMP280();
float cToF( float value );
float mToF( float value );
float findMaximum( float valueArray[], unsigned int size );
float findMinimum( float valueArray[], unsigned int size );
void addValue( float valueArray[], unsigned int size, float value, float minValue, float maxValue );
float averageArray( float valueArray[] );
void toggleLED();
void readTelemetry();
void printTelemetry();
void deviceRestart();
void setup();
void loop();


#endif //ESPSHT30BMP280MQTT_ESPSHT30BMP280MQTT_H
