#ifndef ESPSHT30BMP280MQTT_NETWORK_H
#define ESPSHT30BMP280MQTT_NETWORK_H


#include "EspSht30Bmp280Mqtt.h"
#include <ArduinoOTA.h>   // Arduino Over-The-Air updates.
#include "PubSubClient.h" // MQTT client by Nick O'Leary: https://github.com/knolleary/pubsubclient
#include <WiFiUdp.h>      // Arduino Over-The-Air updates.


#ifdef ESP8266
#include <ESP8266WiFi.h> // ESP8266 WiFi support.  https://github.com/esp8266/Arduino/tree/master/libraries/ESP8266WiFi
#include <ESP8266mDNS.h>
// These two defines accommodate a devkit I have, which powers the onboard LED backwards from what is traditional.
#define LED_ON  0
#define LED_OFF 1
#else
#include <ESPmDNS.h> // Library for multicast DNS, needed for Over-The-Air updates.
#include <WiFi.h>    // Arduino Wi-Fi support.  This header is part of the standard library.  https://www.arduino.cc/en/Reference/WiFi
#define LED_ON  1
#define LED_OFF 0
#endif // ESP8266


const char *COMMAND_TOPIC = "Outdoors/commands";          // The topic used to subscribe to update commands.  Commands: publishTelemetry, changeTelemetryInterval, changeSeaLevelPressure.
const char *TOPIC_PREFIX = "Outdoors/";                   // The MQTT topic prefix, which will have suffixes appended to.
const char *SHT_TEMP_C_TOPIC = "sht30/tempC";             // The MQTT Celsius temperature topic suffix.
const char *SHT_TEMP_F_TOPIC = "sht30/tempF";             // The MQTT Fahrenheit temperature topic suffix.
const char *SHT_HUMIDITY_TOPIC = "sht30/humidity";        // The MQTT humidity topic suffix.
const char *BMP_TEMP_C_TOPIC = "bmp/tempC";               // The BMP Celsius temperature topic suffix.
const char *BMP_TEMP_F_TOPIC = "bmp/tempF";               // The BMP Fahrenheit temperature topic suffix.
const char *BMP_PRESSURE_TOPIC = "bmp/pressure";          // The BMP pressure topic suffix.
const char *RSSI_TOPIC = "rssi";                          // The RSSI topic suffix.
const char *MAC_TOPIC = "mac";                            // The MAC address topic suffix.
const char *IP_TOPIC = "ip";                              // The IP address topic suffix.
const char *WIFI_COUNT_TOPIC = "wifiCount";               // The Wi-Fi count topic suffix.
const char *WIFI_COOLDOWN_TOPIC = "wifiCoolDownInterval"; // The Wi-Fi cooldown count topic suffix.
const char *MQTT_COUNT_TOPIC = "mqttCount";               // The MQTT count topic suffix.
const char *MQTT_COOLDOWN_TOPIC = "mqttCoolDownInterval"; // The MQTT cooldown count topic suffix.
const char *PUBLISH_COUNT_TOPIC = "publishCount";         // The publishCount topic suffix.
const char *MQTT_TOPIC = "espWeather";                    // The topic used to publish a single JSON message containing all data.


WiFiClient wifiClient;
PubSubClient mqttClient( wifiClient );


void lookupWifiCode( int code, char *buffer );
void lookupMQTTCode( int code, char *buffer );
int checkForSSID( const char *ssidName );
bool wifiConnect( const char *ssid, const char *password );
void wifiMultiConnect();
void configureOTA();
void publishTelemetry();
void mqttCallback( char *topic, byte *payload, unsigned int length );
void mqttConnect();


#endif //ESPSHT30BMP280MQTT_NETWORK_H
