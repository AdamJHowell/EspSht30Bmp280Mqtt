#ifdef ESP8266
#include <ESP8266WiFi.h> // ESP8266 WiFi support.  https://github.com/esp8266/Arduino/tree/master/libraries/ESP8266WiFi
#include <ESP8266mDNS.h>
// These two defines accommodate a devkit I have which powers the onboard LED backwards from what is traditional.
#define LED_ON	 0
#define LED_OFF 1
#else
#include "ESPmDNS.h" // Library for multicast DNS, needed for Over-The-Air updates.
#include <WiFi.h>		// Arduino Wi-Fi support.  This header is part of the standard library.  https://www.arduino.cc/en/Reference/WiFi
#define LED_ON	 1
#define LED_OFF 0
#endif

#include "Adafruit_SHT31.h"  // Driver library for the SHT30.  This library includes Wire.h.
#include "privateInfo.h"	  // Location of Wi-Fi and MQTT settings.
#include <Adafruit_BMP280.h> // The Adafruit library for BMP280 sensor.
#include <ArduinoJson.h>	  // ArduinoJson by Benoît Blanchon: https://arduinojson.org/
#include <ArduinoOTA.h>		  // Arduino Over-The-Air updates.
#include <PubSubClient.h>	  // MQTT client by Nick O'Leary: https://github.com/knolleary/pubsubclient
#include <WiFiUdp.h>			  // Arduino Over-The-Air updates.


char ipAddress[16];													 // A character array to hold the IP address.
char macAddress[18];													 // A character array to hold the MAC address, and append a dash and 3 numbers.
long rssi;																 // A global to hold the Received Signal Strength Indicator.
unsigned int printInterval = 7000;								 // How long to wait between telemetry printouts.
unsigned int publishInterval = 20000;							 // How long to wait between telemetry publishes.
unsigned int wifiConnectCount = 0;								 // A counter for how many times the wifiConnect() function has been called.
unsigned int mqttConnectCount = 0;								 // A counter for how many times the mqttConnect() function has been called.
unsigned int invalidValueCount = 0;								 // A counter of how many times invalid values have been measured.
unsigned int publishNow = 0;										 // A flag to indicate that a publish should happen immediately.
unsigned long printCount = 0;										 // A counter of how many times the stats have been printed.
unsigned long publishCount = 0;									 // A counter of how many times the stats have been published.
unsigned long callbackCount = 0;									 // The number of times a callback was received.
unsigned long lastPrintTime = 0;									 // The last time telemetry was printed.
unsigned long lastPublishTime = 0;								 // The last time a MQTT publish was performed.
unsigned long lastWifiConnectTime = 0;							 // The last time a Wi-Fi connection was attempted.
unsigned long lastMqttConnectionTime = 0;						 // The last time a MQTT broker connection was attempted.
unsigned long wifiCoolDownInterval = 10000;					 // How long to wait between Wi-Fi connection attempts.
unsigned long mqttCoolDownInterval = 10000;					 // How long to wait between MQTT broker connection attempts.
unsigned long wifiConnectionTimeout = 15000;					 // The amount of time to wait for a Wi-Fi connection.
unsigned long ledBlinkInterval = 200;							 // The interval between telemetry processing times.
unsigned long lastLedBlinkTime = 0;								 // The time of the last telemetry process.
const unsigned int ONBOARD_LED = 2;								 // The GPIO which the onboard LED is connected to.
const unsigned int JSON_DOC_SIZE = 512;						 // The ArduinoJson document size.
const unsigned int MILLIS_IN_SEC = 1000;						 // The number of milliseconds in one second.
const char *COMMAND_TOPIC = "AdamsEspArmada/commands";	 // The topic used to subscribe to update commands.  Commands: publishTelemetry, changeTelemetryInterval, publishStatus.
const char *TOPIC_PREFIX = "AdamsEspArmada/";				 // The MQTT topic prefix, which will have suffixes appended to.
const char *SHT_TEMP_C_TOPIC = "sht30/tempC";				 // The MQTT Celsius temperature topic suffix.
const char *SHT_TEMP_F_TOPIC = "sht30/tempF";				 // The MQTT Fahrenheit temperature topic suffix.
const char *SHT_HUMIDITY_TOPIC = "sht30/humidity";			 // The MQTT humidity topic suffix.
const char *BMP_TEMP_C_TOPIC = "bmp/tempC";					 // The BMP Celsius temperature topic suffix.
const char *BMP_TEMP_F_TOPIC = "bmp/tempF";					 // The BMP Fahrenheit temperature topic suffix.
const char *BMP_PRESSURE_TOPIC = "bmp/pressure";			 // The BMP pressure topic suffix.
const char *RSSI_TOPIC = "rssi";									 // The RSSI topic suffix.
const char *MAC_TOPIC = "mac";									 // The MAC address topic suffix.
const char *IP_TOPIC = "ip";										 // The IP address topic suffix.
const char *WIFI_COUNT_TOPIC = "wifiCount";					 // The Wi-Fi count topic suffix.
const char *WIFI_COOLDOWN_TOPIC = "wifiCoolDownInterval"; // The Wi-Fi count topic suffix.
const char *MQTT_COUNT_TOPIC = "mqttCount";						 // The MQTT count topic suffix.
const char *MQTT_COOLDOWN_TOPIC = "mqttCoolDownInterval";	 // The MQTT count topic suffix.
const char *PUBLISH_COUNT_TOPIC = "publishCount";				 // The publishCount topic suffix.
const char *MQTT_TOPIC = "espWeather";							 // The topic used to publish a single JSON message containing all data.
float seaLevelPressure = 1026.1;									 // The local sea-level pressure. Provo Airport: https://forecast.weather.gov/data/obhistory/KPVU.html
float sht30TempCArray[] = { 21.12, 21.12, 21.12 };			 // An array to hold the 3 most recent Celsius values.
float sht30HumidityArray[] = { 21.12, 21.12, 21.12 };		 // An array to hold the 3 most recent values.
float bmpTempCArray[] = { -21.12, 21.12, 42.42 };			 // An array to hold the 3 most recent Celsius values, initialized to reasonable levels.
float bmpPressureHPaArray[] = { 8.7, 882.64, 1083.8 };	 // An array to hold the 3 most recent barometric pressure values, initialized to reasonable levels.
float bmpAltitudeMArray[] = { -413.0, 1337.0, 3108.0 };	 // An array to hold the 3 most recent barometric pressure values, initialized to reasonable levels.

//const char *wifiSsid = "nunya";											// Wi-Fi SSID.  Defined in privateInfo.h
//const char *wifiPassword = "nunya";										// Wi-Fi password.  Defined in privateInfo.h
//const char *mqttBroker = "nunya";											// The broker address.  Defined in privateInfo.h
//const unsigned int mqttPort = 1883;										// The broker port.  Defined in privateInfo.h


Adafruit_SHT31 sht30 = Adafruit_SHT31();
Adafruit_BMP280 bmp280;
WiFiClient wifiClient;
PubSubClient mqttClient( wifiClient );

/**
 * @brief setupSht30() will initialize the SHT30 temperature and humidity sensor.
 */
void setupSht30()
{
	unsigned int address = 0x44;
	// Set to 0x45 for alternate i2c address.
	if( !sht30.begin( address ) )
	{
		Serial.printf( "Could not find SHT30 at address %X!\n", address );
		Serial.println( "  Please fix the problem and reboot the device." );
		Serial.println( "  This function is now in an infinite loop." );
		while( 1 )
			delay( 1000 );
	}

	Serial.print( "SHT30 heater state: " );
	if( sht30.isHeaterEnabled() )
		Serial.println( "ENABLED" );
	else
		Serial.println( "DISABLED" );
} // End of the setupSht30() function.

/**
 * @brief setupBMP280() will initialize the sensor and check its status.
 */
void setupBMP280()
{
	Serial.println( "Attempting to connect to the BMP280 at address 0x76..." );
	if( !bmp280.begin( 0x76 ) )
	{
		Serial.println( "Could not find a valid BMP280 sensor!" );
		Serial.println( "Check the wiring and I2C bus settings." );
		Serial.println( "Going into an infinite loop..." );
		while( 1 )
			delay( 3 );
	}
	/* Default settings from datasheet. */
	bmp280.setSampling( Adafruit_BMP280::MODE_NORMAL,		 /* Operating Mode. */
							  Adafruit_BMP280::SAMPLING_X2,		 /* Temp. oversampling */
							  Adafruit_BMP280::SAMPLING_X16,		 /* Pressure oversampling */
							  Adafruit_BMP280::FILTER_X16,		 /* Filtering. */
							  Adafruit_BMP280::STANDBY_MS_500 ); /* Standby time. */

	Serial.println( "Connected to the BMP280!\n" );
} // End of setupBMP280 function.

/**
 * @brief cToF() will convert Celsius to Fahrenheit.
 */
float cToF( float value )
{
	return value * 1.8 + 32;
} // End of the cToF() function.

/**
 * @brief mToF() will convert meters to feet.
 */
float mToF( float value )
{
	return value * 3.28084;
} // End of the mToF() function.

/**
 * @brief addValue() will add the passed value to the 0th element of the passed array, after moving the existing array values to higher indexes.
 * If value is less than minValue, or greater than maxValue, it will be discarded and nothing will be added to valueArray.
 */
void addValue( float valueArray[], unsigned int size, float value, float minValue, float maxValue )
{
	// Prevent sensor anomalies from getting into the array.
	if( value < minValue || value > maxValue )
	{
		Serial.printf( "\n\nValue %f is not between %f and %f!\n\n", value, minValue, maxValue );
		invalidValueCount++;
		return;
	}

	// Detect outliers.
	float minArrayValue = findMinimum( valueArray, size );
	float maxArrayValue = findMaximum( valueArray, size );
	if( value < ( minArrayValue / 2 ) )
	{
		Serial.printf( "\n\nValue %f is less than half the smallest existing value of %f!\n\n", value, minArrayValue );
		invalidValueCount++;
		return;
	}
	if( value > ( maxArrayValue * 2 ) )
	{
		Serial.printf( "\n\nValue %f is more than double the largest existing value of %f!\n\n", value, maxArrayValue );
		invalidValueCount++;
		return;
	}

	valueArray[2] = valueArray[1];
	valueArray[1] = valueArray[0];
	valueArray[0] = value;
} // End of the addValue() function.

/**
 * @brief averageArray() will return the average of values in the passed array.
 */
float averageArray( float valueArray[] )
{
	const unsigned int arraySize = 3;
	float tempValue = 0;
	for( int i = 0; i < arraySize; ++i )
	{
		tempValue += valueArray[i];
	}
	return tempValue / arraySize;
} // End of the averageArray() function.

/**
 * @brief findMaximum() will return the largest value in the passed array.
 */
float findMaximum( float valueArray[], unsigned int size )
{
	float maxValue = valueArray[0];
	for( int i = 1; i < size; ++i )
	{
		if( valueArray[i] > maxValue )
			maxValue = valueArray[i];
	}
	return maxValue;
} // End of the findMaximum() function.

/**
 * @brief findMinimum() will return the smallest value in the passed array.
 */
float findMinimum( float valueArray[], unsigned int size )
{
	float minValue = valueArray[0];
	for( int i = 1; i < size; ++i )
	{
		if( valueArray[i] < minValue )
			minValue = valueArray[i];
	}
	return minValue;
} // End of the findMinimum() function.

/**
 * @brief toggleLED() will change the state of the LED.
 * This function does not manage any timings.
 */
void toggleLED()
{
	if( digitalRead( ONBOARD_LED ) != 1 )
		digitalWrite( ONBOARD_LED, LED_ON );
	else
		digitalWrite( ONBOARD_LED, LED_OFF );
} // End of toggleLED() function.

/**
 * @brief lookupWifiCode() will return the string for an integer code.
 */
void lookupWifiCode( int code, char *buffer )
{
	switch( code )
	{
		case 0:
			snprintf( buffer, 26, "%s", "Idle" );
			break;
		case 1:
			snprintf( buffer, 26, "%s", "No SSID" );
			break;
		case 2:
			snprintf( buffer, 26, "%s", "Scan completed" );
			break;
		case 3:
			snprintf( buffer, 26, "%s", "Connected" );
			break;
		case 4:
			snprintf( buffer, 26, "%s", "Connection failed" );
			break;
		case 5:
			snprintf( buffer, 26, "%s", "Connection lost" );
			break;
		case 6:
			snprintf( buffer, 26, "%s", "Disconnected" );
			break;
		default:
			snprintf( buffer, 26, "%s", "Unknown Wi-Fi status code" );
	}
} // End of the lookupWifiCode() function.

/**
 * @brief lookupMQTTCode() will return the string for an integer state code.
 */
void lookupMQTTCode( int code, char *buffer )
{
	switch( code )
	{
		case -4:
			snprintf( buffer, 29, "%s", "Connection timeout" );
			break;
		case -3:
			snprintf( buffer, 29, "%s", "Connection lost" );
			break;
		case -2:
			snprintf( buffer, 29, "%s", "Connect failed" );
			break;
		case -1:
			snprintf( buffer, 29, "%s", "Disconnected" );
			break;
		case 0:
			snprintf( buffer, 29, "%s", "Connected" );
			break;
		case 1:
			snprintf( buffer, 29, "%s", "Bad protocol" );
			break;
		case 2:
			snprintf( buffer, 29, "%s", "Bad client ID" );
			break;
		case 3:
			snprintf( buffer, 29, "%s", "Unavailable" );
			break;
		case 4:
			snprintf( buffer, 29, "%s", "Bad credentials" );
			break;
		case 5:
			snprintf( buffer, 29, "%s", "Unauthorized" );
			break;
		default:
			snprintf( buffer, 29, "%s", "Unknown MQTT state code" );
	}
} // End of the lookupMQTTCode() function.

/**
 * @brief checkForSSID() will scan for all visible SSIDs, see if any match 'ssidName',
 * and return a count of how many matches were found.
 *
 * @param ssidName the SSID name to search for.
 * @return int the count of SSIDs which match the passed parameter.
 */
int checkForSSID( const char *ssidName )
{
	int ssidCount = 0;
	byte networkCount = WiFi.scanNetworks();
	if( networkCount == 0 )
		Serial.println( "No WiFi SSIDs are in range!" );
	else
	{
		Serial.printf( "WiFi SSIDs in range: %d\n", networkCount );
		for( int i = 0; i < networkCount; ++i )
		{
			// Check to see if this SSID matches the parameter.
			if( strcmp( ssidName, WiFi.SSID( i ).c_str() ) == 0 )
				ssidCount++;
		}
	}
	return ssidCount;
} // End of checkForSSID() function.

/**
 * @brief wifiConnect() will connect to a SSID.
 */
void wifiConnect()
{
	long time = millis();
	if( lastWifiConnectTime == 0 || ( time > wifiCoolDownInterval && ( time - wifiCoolDownInterval ) > lastWifiConnectTime ) )
	{
		int ssidCount = checkForSSID( wifiSsid );
		if( ssidCount == 0 )
		{
			Serial.printf( "SSID '%s' is not in range!\n", wifiSsid );
			digitalWrite( ONBOARD_LED, LED_OFF ); // Turn the LED off to show that Wi-Fi has no chance of connecting.
		}
		else
		{
			wifiConnectCount++;
			// Turn the LED off to show Wi-Fi is not connected.
			digitalWrite( ONBOARD_LED, LED_OFF );

			Serial.printf( "Attempting to connect to Wi-Fi SSID '%s'", wifiSsid );
			WiFi.mode( WIFI_STA );
			//			WiFi.config( INADDR_NONE, INADDR_NONE, INADDR_NONE, INADDR_NONE );
			const char *hostName = macAddress;
			WiFi.setHostname( hostName );
			WiFi.begin( wifiSsid, wifiPassword );

			unsigned long wifiConnectionStartTime = millis();

			// Loop until connected, or until wifiConnectionTimeout.
			while( WiFi.status() != WL_CONNECTED && ( millis() - wifiConnectionStartTime < wifiConnectionTimeout ) )
			{
				Serial.print( "." );
				delay( 1000 );
			}
			Serial.println( "" );

			if( WiFi.status() == WL_CONNECTED )
			{
				// Print that Wi-Fi has connected.
				Serial.println( "\nWi-Fi connection established!" );
				snprintf( ipAddress, 16, "%d.%d.%d.%d", WiFi.localIP()[0], WiFi.localIP()[1], WiFi.localIP()[2], WiFi.localIP()[3] );
				// Turn the LED on to show that Wi-Fi is connected.
				digitalWrite( ONBOARD_LED, LED_ON );
				return;
			}
			else
				Serial.println( "Wi-Fi failed to connect in the timeout period.\n" );
		}
		lastWifiConnectTime = millis();
	}
} // End of the wifiConnect() function.

/**
 * @brief configureOTA() will configure and initiate Over The Air (OTA) updates for this device.
 */
void configureOTA()
{
	Serial.println( "Configuring OTA." );

#ifdef ESP8266
	// The ESP8266 port defaults to 8266
	// ArduinoOTA.setPort( 8266 );
	// The ESP8266 hostname defaults to esp8266-[ChipID]
	// ArduinoOTA.setHostname( hostName );
	// Authentication is disabled by default.
	// ArduinoOTA.setPassword( ( const char * )"admin" );
	ArduinoOTA.onStart( []() {
		String type;
		if( ArduinoOTA.getCommand() == U_FLASH )
			type = "sketch";
		else // U_SPIFFS
			type = "filesystem";

		// NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
		Serial.println( "Start updating " + type );
	} );
#else
	// The ESP32 port defaults to 3232
	// ArduinoOTA.setPort( 3232 );
	// The ESP32 hostname defaults to esp32-[MAC]
	//	ArduinoOTA.setHostname( hostName );  // I'm deliberately using the default.
	// Authentication is disabled by default.
	// ArduinoOTA.setPassword( "admin" );
	// Password can be set with it's md5 value as well
	// MD5( admin ) = 21232f297a57a5a743894a0e4a801fc3
	// ArduinoOTA.setPasswordHash( "21232f297a57a5a743894a0e4a801fc3" );

	//	Serial.printf( "Using hostname '%s'\n", hostName );

	String type = "filesystem"; // SPIFFS
	if( ArduinoOTA.getCommand() == U_FLASH )
		type = "sketch";

	// Configure the OTA callbacks.
	ArduinoOTA.onStart( []() {
		String type = "flash"; // U_FLASH
		if( ArduinoOTA.getCommand() == U_SPIFFS )
			type = "filesystem";
		// NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
		Serial.print( "OTA is updating the " );
		Serial.println( type );
	} );
#endif
	ArduinoOTA.onEnd( []() { Serial.println( "\nTerminating OTA communication." ); } );
	ArduinoOTA.onProgress( []( unsigned int progress, unsigned int total ) { Serial.printf( "OTA progress: %u%%\r", ( progress / ( total / 100 ) ) ); } );
	ArduinoOTA.onError( []( ota_error_t error ) {
		Serial.printf( "Error[%u]: ", error );
		if( error == OTA_AUTH_ERROR ) Serial.println( "OTA authentication failed!" );
		else if( error == OTA_BEGIN_ERROR ) Serial.println( "OTA transmission failed to initiate properly!" );
		else if( error == OTA_CONNECT_ERROR ) Serial.println( "OTA connection failed!" );
		else if( error == OTA_RECEIVE_ERROR ) Serial.println( "OTA client was unable to properly receive data!" );
		else if( error == OTA_END_ERROR ) Serial.println( "OTA transmission failed to terminate properly!" ); } );

	// Start listening for OTA commands.
	ArduinoOTA.begin();

	Serial.println( "OTA is configured and ready." );
} // End of the configureOTA() function.

/**
 * @brief readTelemetry() will read the telemetry and save values to global variables.
 */
void readTelemetry()
{
	rssi = WiFi.RSSI();
	// Add current readings into the appropriate arrays.
	addValue( sht30TempCArray, 3, sht30.readTemperature(), -42, 212 );
	addValue( sht30HumidityArray, 3, sht30.readHumidity(), 0, 100 );

	// Get readings from the BMP280.
	addValue( bmpTempCArray, 3, bmp280.readTemperature(), -30, 80 );
	addValue( bmpPressureHPaArray, 3, bmp280.readPressure() / 100, 300, 1500 );
	addValue( bmpAltitudeMArray, 3, bmp280.readAltitude( seaLevelPressure ), 0, 10000 );
} // End of the readTelemetry() function.

/**
 * @brief printTelemetry() will print the telemetry to the serial port.
 */
void printTelemetry()
{
	Serial.println();
	printCount++;
	Serial.println( __FILE__ );
	Serial.printf( "Print count %ld\n", printCount );
	Serial.println();

	Serial.println( "Network stats:" );
	Serial.printf( "  MAC address: %s\n", macAddress );
	int wifiStatusCode = WiFi.status();
	char buffer[29];
	lookupWifiCode( wifiStatusCode, buffer );
	if( wifiStatusCode == 3 )
	{
		Serial.printf( "  IP address: %s\n", ipAddress );
		Serial.printf( "  RSSI: %ld\n", rssi );
		Serial.print( "~~IP address: " );
		Serial.println( WiFi.localIP() );
	}
	Serial.printf( "  wifiConnectCount: %u\n", wifiConnectCount );
	Serial.printf( "  wifiCoolDownInterval: %lu\n", wifiCoolDownInterval );
	Serial.printf( "  Wi-Fi status text: %s\n", buffer );
	Serial.printf( "  Wi-Fi status code: %d\n", wifiStatusCode );
	Serial.println();

	Serial.println( "MQTT stats:" );
	Serial.printf( "  mqttConnectCount: %u\n", mqttConnectCount );
	Serial.printf( "  mqttCoolDownInterval: %lu\n", mqttCoolDownInterval );
	Serial.printf( "  Broker: %s:%d\n", mqttBroker, mqttPort );
	lookupMQTTCode( mqttClient.state(), buffer );
	Serial.printf( "  MQTT state: %s\n", buffer );
	Serial.printf( "  Publish count: %lu\n", publishCount );
	Serial.printf( "  Callback count: %lu\n", callbackCount );
	Serial.println();

	Serial.println( "Environmental stats:" );
	Serial.printf( "  SHT30 tempC: %f\n", averageArray( sht30TempCArray ) );
	Serial.printf( "  SHT30 tempF: %f\n", cToF( averageArray( sht30TempCArray ) ) );
	Serial.printf( "  SHT30 humidity: %f\n", averageArray( sht30HumidityArray ) );
	Serial.printf( "  BMP280 temperature: %.2f C\n", averageArray( bmpTempCArray ) );
	Serial.printf( "  BMP280 temperature: %.2f F\n", cToF( averageArray( bmpTempCArray ) ) );
	Serial.printf( "  BMP280 pressure: %.2f hPa\n", averageArray( bmpPressureHPaArray ) );
	Serial.printf( "  BMP280 altitude: %.2f m\n", averageArray( bmpAltitudeMArray ) );
	Serial.printf( "  BMP280 altitude: %.2f f\n", mToF( averageArray( bmpAltitudeMArray ) ) );
	Serial.printf( "  Sea level pressure: %.2f hPa\n", seaLevelPressure );
	Serial.printf( "  Invalid reading count from all sensors: %u\n", invalidValueCount );
	Serial.println();

	Serial.printf( "Next telemetry poll in %lu seconds\n", printInterval / MILLIS_IN_SEC );
	Serial.println( "\n" );
} // End of the printTelemetry() function.

/**
 * @brief publishTelemetry() will process incoming messages on subscribed topics.
 */
void publishTelemetry()
{
	// Create a JSON Document on the stack.
	StaticJsonDocument<JSON_DOC_SIZE> publishTelemetryJsonDoc;
	// Add data: __FILE__, macAddress, ipAddress, tempC, tempF, soilMoisture, rssi, publishCount, notes
	publishTelemetryJsonDoc["sketch"] = __FILE__;
	publishTelemetryJsonDoc["mac"] = macAddress;
	publishTelemetryJsonDoc["ip"] = ipAddress;
	publishTelemetryJsonDoc["tempC"] = averageArray( sht30TempCArray );
	publishTelemetryJsonDoc["tempF"] = cToF( averageArray( sht30TempCArray ) );
	publishTelemetryJsonDoc["rssi"] = rssi;
	publishTelemetryJsonDoc["publishCount"] = publishCount;
	publishTelemetryJsonDoc["invalidValueCount"] = invalidValueCount;
	// Prepare a String to hold the JSON.
	char mqttString[JSON_DOC_SIZE];
	// Serialize the JSON into mqttString, with indentation and line breaks.
	serializeJsonPretty( publishTelemetryJsonDoc, mqttString );
	// Publish the JSON to the MQTT broker.
	bool success = mqttClient.publish( MQTT_TOPIC, mqttString, false );
	if( success )
		Serial.printf( "Successfully published to '%s'\n", MQTT_TOPIC );

	publishCount++;
	char topicBuffer[256] = "";
	char valueBuffer[25] = "";

	snprintf( topicBuffer, 256, "%s%s%s%s", TOPIC_PREFIX, macAddress, "/", SHT_TEMP_C_TOPIC );
	snprintf( valueBuffer, 25, "%f", averageArray( sht30TempCArray ) );
	Serial.printf( "Publishing '%s' to '%s'\n", valueBuffer, topicBuffer );
	mqttClient.publish( topicBuffer, valueBuffer );

	snprintf( topicBuffer, 256, "%s%s%s%s", TOPIC_PREFIX, macAddress, "/", SHT_TEMP_F_TOPIC );
	snprintf( valueBuffer, 25, "%f", cToF( averageArray( sht30TempCArray ) ) );
	Serial.printf( "Publishing '%s' to '%s'\n", valueBuffer, topicBuffer );
	mqttClient.publish( topicBuffer, valueBuffer );

	snprintf( topicBuffer, 256, "%s%s%s%s", TOPIC_PREFIX, macAddress, "/", SHT_HUMIDITY_TOPIC );
	snprintf( valueBuffer, 25, "%f", averageArray( sht30HumidityArray ) );
	Serial.printf( "Publishing '%s' to '%s'\n", valueBuffer, topicBuffer );
	mqttClient.publish( topicBuffer, valueBuffer );

	snprintf( topicBuffer, 256, "%s%s%s%s", TOPIC_PREFIX, macAddress, "/", BMP_TEMP_C_TOPIC );
	snprintf( valueBuffer, 25, "%f", averageArray( bmpTempCArray ) );
	Serial.printf( "Publishing '%s' to '%s'\n", valueBuffer, topicBuffer );
	mqttClient.publish( topicBuffer, valueBuffer );

	snprintf( topicBuffer, 256, "%s%s%s%s", TOPIC_PREFIX, macAddress, "/", BMP_TEMP_F_TOPIC );
	snprintf( valueBuffer, 25, "%f", cToF( averageArray( bmpTempCArray ) ) );
	Serial.printf( "Publishing '%s' to '%s'\n", valueBuffer, topicBuffer );
	mqttClient.publish( topicBuffer, valueBuffer );

	snprintf( topicBuffer, 256, "%s%s%s%s", TOPIC_PREFIX, macAddress, "/", BMP_PRESSURE_TOPIC );
	snprintf( valueBuffer, 25, "%f", averageArray( bmpPressureHPaArray ) );
	Serial.printf( "Publishing '%s' to '%s'\n", valueBuffer, topicBuffer );
	mqttClient.publish( topicBuffer, valueBuffer );

	snprintf( topicBuffer, 256, "%s%s%s%s", TOPIC_PREFIX, macAddress, "/", RSSI_TOPIC );
	snprintf( valueBuffer, 25, "%ld", rssi );
	Serial.printf( "Publishing '%s' to '%s'\n", valueBuffer, topicBuffer );
	mqttClient.publish( topicBuffer, valueBuffer );

	snprintf( topicBuffer, 256, "%s%s%s%s", TOPIC_PREFIX, macAddress, "/", MAC_TOPIC );
	snprintf( valueBuffer, 25, "%s", macAddress );
	Serial.printf( "Publishing '%s' to '%s'\n", valueBuffer, topicBuffer );
	mqttClient.publish( topicBuffer, valueBuffer );

	snprintf( topicBuffer, 256, "%s%s%s%s", TOPIC_PREFIX, macAddress, "/", IP_TOPIC );
	snprintf( valueBuffer, 25, "%s", ipAddress );
	Serial.printf( "Publishing '%s' to '%s'\n", valueBuffer, topicBuffer );
	mqttClient.publish( topicBuffer, valueBuffer );

	snprintf( topicBuffer, 256, "%s%s%s%s", TOPIC_PREFIX, macAddress, "/", WIFI_COUNT_TOPIC );
	snprintf( valueBuffer, 25, "%u", wifiConnectCount );
	Serial.printf( "Publishing '%s' to '%s'\n", valueBuffer, topicBuffer );
	mqttClient.publish( topicBuffer, valueBuffer );

	snprintf( topicBuffer, 256, "%s%s%s%s", TOPIC_PREFIX, macAddress, "/", WIFI_COOLDOWN_TOPIC );
	snprintf( valueBuffer, 25, "%lu", wifiCoolDownInterval );
	Serial.printf( "Publishing '%s' to '%s'\n", valueBuffer, topicBuffer );
	mqttClient.publish( topicBuffer, valueBuffer );

	snprintf( topicBuffer, 256, "%s%s%s%s", TOPIC_PREFIX, macAddress, "/", MQTT_COUNT_TOPIC );
	snprintf( valueBuffer, 25, "%u", mqttConnectCount );
	Serial.printf( "Publishing '%s' to '%s'\n", valueBuffer, topicBuffer );
	mqttClient.publish( topicBuffer, valueBuffer );

	snprintf( topicBuffer, 256, "%s%s%s%s", TOPIC_PREFIX, macAddress, "/", MQTT_COOLDOWN_TOPIC );
	snprintf( valueBuffer, 25, "%lu", mqttCoolDownInterval );
	Serial.printf( "Publishing '%s' to '%s'\n", valueBuffer, topicBuffer );
	mqttClient.publish( topicBuffer, valueBuffer );

	snprintf( topicBuffer, 256, "%s%s%s%s", TOPIC_PREFIX, macAddress, "/", PUBLISH_COUNT_TOPIC );
	snprintf( valueBuffer, 25, "%lu", publishCount );
	Serial.printf( "Publishing '%s' to '%s'\n", valueBuffer, topicBuffer );
	mqttClient.publish( topicBuffer, valueBuffer );
} // End of the publishTelemetry() function.

/**
 * @brief mqttCallback() will process incoming messages on subscribed topics.
 * { "command": "publishTelemetry" }
 * { "command": "changeTelemetryInterval", "value": 10000 }
 * ToDo: Add more commands for the board to react to.
 */
void mqttCallback( char *topic, byte *payload, unsigned int length )
{
	callbackCount++;
	Serial.printf( "\nMessage arrived on Topic: '%s'\n", topic );

	StaticJsonDocument<JSON_DOC_SIZE> callbackJsonDoc;
	Serial.println( "JSON document (static) was created." );
	deserializeJson( callbackJsonDoc, payload, length );
	Serial.println( "JSON document was deserialized." );

	const char *command = callbackJsonDoc["command"];
	Serial.printf( "Processing command '%s'.\n", command );
	if( strcmp( command, "publishTelemetry" ) == 0 )
	{
		// Set publishNow to 1, to indicate that a telemetry read and publish should happen immediately.
		publishNow = 1;
	}
	else if( strcmp( command, "changeTelemetryInterval" ) == 0 )
	{
		unsigned long tempValue = callbackJsonDoc["value"];
		// Only update the value if it is greater than 4 seconds.  This prevents a seconds vs. milliseconds confusion.
		if( tempValue > 4000 )
			publishInterval = tempValue;
		Serial.printf( "MQTT publish interval has been updated to %u\n", publishInterval );
		lastPublishTime = 0;
	}
	else if( strcmp( command, "publishStatus" ) == 0 )
		Serial.println( "publishStatus is not yet implemented." );
	else
		Serial.printf( "Unknown command '%s'\n", command );
} // End of the mqttCallback() function.

/**
 * @brief mqttConnect() will connect to the MQTT broker.
 */
void mqttConnect()
{
	long time = millis();
	// Connect the first time.  Avoid subtraction overflow.  Connect after cool down.
	if( lastMqttConnectionTime == 0 || ( time > mqttCoolDownInterval && ( time - mqttCoolDownInterval ) > lastMqttConnectionTime ) )
	{
		mqttConnectCount++;
		digitalWrite( ONBOARD_LED, LED_OFF );
		Serial.printf( "Connecting to broker at %s:%d...\n", mqttBroker, mqttPort );
		mqttClient.setServer( mqttBroker, mqttPort );
		mqttClient.setCallback( mqttCallback );

		// Connect to the broker, using the MAC address for a MQTT client ID.
		if( mqttClient.connect( macAddress ) )
		{
			Serial.println( "Connected to MQTT Broker." );
			mqttClient.subscribe( COMMAND_TOPIC );
			digitalWrite( ONBOARD_LED, LED_ON );
		}
		else
		{
			int mqttStateCode = mqttClient.state();
			char buffer[29];
			lookupMQTTCode( mqttStateCode, buffer );
			Serial.printf( "MQTT state: %s\n", buffer );
			Serial.printf( "MQTT state code: %d\n", mqttStateCode );

			// This block increments the broker connection "cooldown" time by 10 seconds after every failed connection, and resets it once it is over 2 minutes.
			if( mqttCoolDownInterval > 120000 )
				mqttCoolDownInterval = 0;
			mqttCoolDownInterval += 10000;
		}

		lastMqttConnectionTime = millis();
	}
} // End of the mqttConnect() function.

/**
 * @brief setup() will configure the program.
 */
void setup()
{
	delay( 1000 );
	Serial.begin( 115200 );
	if( !Serial )
		delay( 1000 );
	Serial.println( "\n" );
	Serial.println( "Function setup() is beginning." );

	// Set the MAC address variable to its value.
	snprintf( macAddress, 18, "%s", WiFi.macAddress().c_str() );

	// Set ONBOARD_LED (GPIO 2) as an output.
	pinMode( ONBOARD_LED, OUTPUT );
	// Turn the LED on to show that setup() has begun.
	digitalWrite( ONBOARD_LED, LED_ON );

	setupSht30();
	setupBMP280();

	// Read from the sensors twice, to populate telemetry arrays.
	readTelemetry();
	readTelemetry();

	wifiConnect();
	configureOTA();

	Serial.println( "Function setup() has completed." );
} // End of the setup() function.

/**
 * @brief loop() repeats over and over.
 */
void loop()
{
	// Reconnect Wi-Fi if needed, reconnect MQTT as needed.
	if( WiFi.status() != WL_CONNECTED )
		wifiConnect();
	else if( !mqttClient.connected() )
		mqttConnect();
	else
	{
		mqttClient.loop();
		ArduinoOTA.handle();
	}

	long currentTime = millis();
	// Print the first time.  Avoid subtraction overflow.  Print every interval.
	if( lastPrintTime == 0 || ( currentTime > printInterval && ( currentTime - printInterval ) > lastPrintTime ) )
	{
		readTelemetry();
		printTelemetry();
		Serial.printf( "Next print in %u seconds.\n\n", printInterval / 1000 );
		lastPrintTime = millis();
	}

	currentTime = millis();
	// Publish only if connected.  Publish the first time.  Avoid subtraction overflow.  Publish every interval.
	if( mqttClient.connected() && ( publishNow == 1 || lastPublishTime == 0 || ( currentTime > publishInterval && ( currentTime - publishInterval ) > lastPublishTime ) ) )
	{
		if( publishNow == 1 )
			readTelemetry();
		publishTelemetry();
		lastPublishTime = millis();
		publishNow = 0;
		Serial.printf( "Next publish in %u seconds.\n\n", publishInterval / 1000 );
		lastPublishTime = millis();
	}

	currentTime = millis();
	// Process the first time.  Avoid subtraction overflow.  Process every interval.
	if( lastLedBlinkTime == 0 || ( ( currentTime > ledBlinkInterval ) && ( currentTime - ledBlinkInterval ) > lastLedBlinkTime ) )
	{
		// If Wi-Fi is connected, but MQTT is not, blink the LED.
		if( WiFi.status() == WL_CONNECTED )
		{
			if( mqttClient.state() != 0 )
				toggleLED(); // Toggle the LED state to show that Wi-Fi is connected by MQTT is not.
			else
				digitalWrite( ONBOARD_LED, LED_ON ); // Turn the LED on to show both Wi-Fi and MQTT are connected.
		}
		else
			digitalWrite( ONBOARD_LED, LED_OFF ); // Turn the LED off to show that Wi-Fi is not connected.
		lastLedBlinkTime = millis();
	}

	// Reset the device if the number of invalid sensor readings is too high.
	if( invalidValueCount > 10 )
	{
		Serial.println( "\n\n\n" );
		Serial.println( "Too many invalid sensor readings have occurred!" );
		Serial.println( "The device will reset in 5 seconds!" );
		delay( 5 * MILLIS_IN_SEC );
		Serial.println( "\n\n\n" );
		ESP.restart();
	}
} // End of the loop() function.