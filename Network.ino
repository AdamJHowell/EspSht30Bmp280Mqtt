#include "Network.h"

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
 * The string values are taken from PubSubClient.h
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
      //      Serial.printf( "WiFi SSIDs in range: %d\n", networkCount );
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
 * @brief wifiConnect() will attempt to connect to a single SSID.
 */
bool wifiConnect( const char *ssid, const char *password )
{
   wifiConnectCount++;
   // Turn the LED off to show Wi-Fi is not connected.
   digitalWrite( ONBOARD_LED, LED_OFF );

   Serial.printf( "Attempting to connect to Wi-Fi SSID '%s'", ssid );
   WiFi.mode( WIFI_STA );
   //   WiFi.setHostname( hostName );
   WiFi.begin( ssid, password );

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
      Serial.println( "Wi-Fi connection established!" );
      snprintf( ipAddress, 16, "%d.%d.%d.%d", WiFi.localIP()[0], WiFi.localIP()[1], WiFi.localIP()[2], WiFi.localIP()[3] );
      // Turn the LED on to show that Wi-Fi is connected.
      digitalWrite( ONBOARD_LED, LED_ON );
      return true;
   }
   Serial.println( "Wi-Fi failed to connect in the timeout period.\n" );
   return false;
} // End of the wifiConnect() function.

/**
 * @brief wifiMultiConnect() will iterate through the SSIDs in 'wifiSsidArray[]', and then use checkForSSID() determine which are in range.
 * When a SSID is in range, wifiConnect() will be called with that SSID and password.
 */
void wifiMultiConnect()
{
   long time = millis();
   if( lastWifiConnectTime == 0 || ( time > wifiCoolDownInterval && ( time - wifiCoolDownInterval ) > lastWifiConnectTime ) )
   {
      Serial.println( "\nEntering wifiMultiConnect()" );
      digitalWrite( ONBOARD_LED, LED_OFF ); // Turn the LED off to show that Wi-Fi is not yet connected.
      unsigned int arraySize = sizeof( wifiSsidArray );
      for( unsigned int networkArrayIndex = 0; networkArrayIndex < arraySize; networkArrayIndex++ )
      {
         // Get the details for this connection attempt.
         const char *wifiSsid = wifiSsidArray[networkArrayIndex];
         const char *wifiPassword = wifiPassArray[networkArrayIndex];

         // Announce the Wi-Fi parameters for this connection attempt.
         Serial.print( "Attempting to connect to to SSID \"" );
         Serial.print( wifiSsid );
         Serial.println( "\"" );

         // Don't even try to connect if the SSID cannot be found.
         int ssidCount = checkForSSID( wifiSsid );
         if( ssidCount < 1 )
         {
            Serial.printf( "Network '%s' was not found!\n\n", wifiSsid );
            return;
         }
         // This is useful for detecting multiples APs.
         if( ssidCount > 1 )
            Serial.printf( "Found %d SSIDs matching '%s'.\n", ssidCount, wifiSsid );

         // If the Wi-Fi connection is successful, set the mqttClient broker parameters.
         if( wifiConnect( wifiSsid, wifiPassword ) )
         {
            mqttClient.setServer( mqttBrokerArray[networkArrayIndex], mqttPortArray[networkArrayIndex] );
            break;
         }
      }
      Serial.println( "Exiting wifiMultiConnect()\n" );
      lastWifiConnectTime = millis();
   }
} // End of wifiMultiConnect() function.

/**
 * @brief configureOTA() will configure and initiate Over The Air (OTA) updates for this device.
 */
void configureOTA()
{
#ifdef ESP8266
   Serial.println( "Configuring OTA for the ESP8266" );
   // Port defaults to 8266
   // ArduinoOTA.setPort(8266);

   // Hostname defaults to esp8266-[ChipID]
   ArduinoOTA.setHostname( hostName );

   // No authentication by default
   ArduinoOTA.setPassword( otaPass );

   // Password can be set with it's md5 value as well
   // MD5(admin) = 21232f297a57a5a743894a0e4a801fc3
   // ArduinoOTA.setPasswordHash("21232f297a57a5a743894a0e4a801fc3");

   ArduinoOTA.onStart( []() {
      String type;
      if( ArduinoOTA.getCommand() == U_FLASH )
         type = "sketch";
      else // U_SPIFFS
         type = "filesystem";

      // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
      Serial.println( "Start updating " + type );
   } );
   ArduinoOTA.onEnd( []() {
      Serial.println( "\nEnd" );
   } );
   ArduinoOTA.onProgress( []( unsigned int progress, unsigned int total ) {
      Serial.printf( "Progress: %u%%\r", ( progress / ( total / 100 ) ) );
   } );
   ArduinoOTA.onError( []( ota_error_t error ) {
      Serial.printf( "Error[%u]: ", error );
      if( error == OTA_AUTH_ERROR ) Serial.println( "Auth Failed" );
      else if( error == OTA_BEGIN_ERROR )
         Serial.println( "Begin Failed" );
      else if( error == OTA_CONNECT_ERROR )
         Serial.println( "Connect Failed" );
      else if( error == OTA_RECEIVE_ERROR )
         Serial.println( "Receive Failed" );
      else if( error == OTA_END_ERROR )
         Serial.println( "End Failed" );
   } );
#else
   Serial.println( "Configuring OTA for the ESP32" );
   // The ESP32 port defaults to 3232
   // ArduinoOTA.setPort( 3232 );
   // The ESP32 hostname defaults to esp32-[MAC]
   //	ArduinoOTA.setHostname( hostName );  // I'm deliberately using the default.
   // Authentication is disabled by default.
   ArduinoOTA.setPassword( otaPass );
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
   ArduinoOTA.onEnd( []() { Serial.println( "\nTerminating OTA communication." ); } );
   ArduinoOTA.onProgress( []( unsigned int progress, unsigned int total ) { Serial.printf( "OTA progress: %u%%\r", ( progress / ( total / 100 ) ) ); } );
   ArduinoOTA.onError( []( ota_error_t error ) {
		Serial.printf( "Error[%u]: ", error );
		if( error == OTA_AUTH_ERROR ) Serial.println( "OTA authentication failed!" );
		else if( error == OTA_BEGIN_ERROR ) Serial.println( "OTA transmission failed to initiate properly!" );
		else if( error == OTA_CONNECT_ERROR ) Serial.println( "OTA connection failed!" );
		else if( error == OTA_RECEIVE_ERROR ) Serial.println( "OTA client was unable to properly receive data!" );
		else if( error == OTA_END_ERROR ) Serial.println( "OTA transmission failed to terminate properly!" ); } );
#endif

   // Start listening for OTA commands.
   ArduinoOTA.begin();

   Serial.println( "OTA is configured and ready." );
} // End of the configureOTA() function.

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
      Serial.println( "Connecting to the MQTT broker." );

      // setServer() call is now called in wifiMultiConnect().
      //mqttClient.setServer( mqttBroker, mqttPort );
      mqttClient.setCallback( mqttCallback );

      // Connect to the broker, using the MAC address as the client ID.
      if( mqttClient.connect( macAddress ) )
      {
         Serial.println( "Connected to the broker." );
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
 * @brief publishTelemetry() will process incoming messages on subscribed topics.
 */
void publishTelemetry()
{
   publishCount++;
   char topicBuffer[256] = "";
   char valueBuffer[25] = "";

   snprintf( topicBuffer, 256, "%s%s%s%s", TOPIC_PREFIX, macAddress, "/", SHT_TEMP_C_TOPIC );
   snprintf( valueBuffer, 25, "%f", averageArray( sht30TempCArray ) );
   if( mqttClient.publish( topicBuffer, valueBuffer ) )
      Serial.printf( "Published '%s' to '%s'\n", valueBuffer, topicBuffer );

   snprintf( topicBuffer, 256, "%s%s%s%s", TOPIC_PREFIX, macAddress, "/", SHT_TEMP_F_TOPIC );
   snprintf( valueBuffer, 25, "%f", cToF( averageArray( sht30TempCArray ) ) );
   if( mqttClient.publish( topicBuffer, valueBuffer ) )
      Serial.printf( "Publishing '%s' to '%s'\n", valueBuffer, topicBuffer );

   snprintf( topicBuffer, 256, "%s%s%s%s", TOPIC_PREFIX, macAddress, "/", SHT_HUMIDITY_TOPIC );
   snprintf( valueBuffer, 25, "%f", averageArray( sht30HumidityArray ) );
   if( mqttClient.publish( topicBuffer, valueBuffer ) )
      Serial.printf( "Publishing '%s' to '%s'\n", valueBuffer, topicBuffer );

   snprintf( topicBuffer, 256, "%s%s%s%s", TOPIC_PREFIX, macAddress, "/", BMP_TEMP_C_TOPIC );
   snprintf( valueBuffer, 25, "%f", averageArray( bmpTempCArray ) );
   if( mqttClient.publish( topicBuffer, valueBuffer ) )
      Serial.printf( "Publishing '%s' to '%s'\n", valueBuffer, topicBuffer );

   snprintf( topicBuffer, 256, "%s%s%s%s", TOPIC_PREFIX, macAddress, "/", BMP_TEMP_F_TOPIC );
   snprintf( valueBuffer, 25, "%f", cToF( averageArray( bmpTempCArray ) ) );
   if( mqttClient.publish( topicBuffer, valueBuffer ) )
      Serial.printf( "Publishing '%s' to '%s'\n", valueBuffer, topicBuffer );

   snprintf( topicBuffer, 256, "%s%s%s%s", TOPIC_PREFIX, macAddress, "/", BMP_PRESSURE_TOPIC );
   snprintf( valueBuffer, 25, "%f", averageArray( bmpPressureHPaArray ) );
   if( mqttClient.publish( topicBuffer, valueBuffer ) )
      Serial.printf( "Publishing '%s' to '%s'\n", valueBuffer, topicBuffer );

   snprintf( topicBuffer, 256, "%s%s%s%s", TOPIC_PREFIX, macAddress, "/", RSSI_TOPIC );
   snprintf( valueBuffer, 25, "%ld", rssi );
   if( mqttClient.publish( topicBuffer, valueBuffer ) )
      Serial.printf( "Publishing '%s' to '%s'\n", valueBuffer, topicBuffer );

   snprintf( topicBuffer, 256, "%s%s%s%s", TOPIC_PREFIX, macAddress, "/", MAC_TOPIC );
   snprintf( valueBuffer, 25, "%s", macAddress );
   if( mqttClient.publish( topicBuffer, valueBuffer ) )
      Serial.printf( "Publishing '%s' to '%s'\n", valueBuffer, topicBuffer );

   snprintf( topicBuffer, 256, "%s%s%s%s", TOPIC_PREFIX, macAddress, "/", IP_TOPIC );
   snprintf( valueBuffer, 25, "%s", ipAddress );
   if( mqttClient.publish( topicBuffer, valueBuffer ) )
      Serial.printf( "Publishing '%s' to '%s'\n", valueBuffer, topicBuffer );

   snprintf( topicBuffer, 256, "%s%s%s%s", TOPIC_PREFIX, macAddress, "/", WIFI_COUNT_TOPIC );
   snprintf( valueBuffer, 25, "%u", wifiConnectCount );
   if( mqttClient.publish( topicBuffer, valueBuffer ) )
      Serial.printf( "Publishing '%s' to '%s'\n", valueBuffer, topicBuffer );

   snprintf( topicBuffer, 256, "%s%s%s%s", TOPIC_PREFIX, macAddress, "/", WIFI_COOLDOWN_TOPIC );
   snprintf( valueBuffer, 25, "%lu", wifiCoolDownInterval );
   if( mqttClient.publish( topicBuffer, valueBuffer ) )
      Serial.printf( "Publishing '%s' to '%s'\n", valueBuffer, topicBuffer );

   snprintf( topicBuffer, 256, "%s%s%s%s", TOPIC_PREFIX, macAddress, "/", MQTT_COUNT_TOPIC );
   snprintf( valueBuffer, 25, "%u", mqttConnectCount );
   if( mqttClient.publish( topicBuffer, valueBuffer ) )
      Serial.printf( "Publishing '%s' to '%s'\n", valueBuffer, topicBuffer );

   snprintf( topicBuffer, 256, "%s%s%s%s", TOPIC_PREFIX, macAddress, "/", MQTT_COOLDOWN_TOPIC );
   snprintf( valueBuffer, 25, "%lu", mqttCoolDownInterval );
   if( mqttClient.publish( topicBuffer, valueBuffer ) )
      Serial.printf( "Publishing '%s' to '%s'\n", valueBuffer, topicBuffer );

   snprintf( topicBuffer, 256, "%s%s%s%s", TOPIC_PREFIX, macAddress, "/", PUBLISH_COUNT_TOPIC );
   snprintf( valueBuffer, 25, "%lu", publishCount );
   if( mqttClient.publish( topicBuffer, valueBuffer ) )
      Serial.printf( "Publishing '%s' to '%s'\n", valueBuffer, topicBuffer );
} // End of the publishTelemetry() function.

/**
 * @brief mqttCallback() will process incoming messages on subscribed topics.
 * { "command": "publishTelemetry" }
 * { "command": "changeTelemetryInterval", "value": 10000 }
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
   else if( strcmp( command, "changeSeaLevelPressure" ) == 0 )
   {
      unsigned long tempValue = callbackJsonDoc["value"];
      if( tempValue > 800 && tempValue < 1200 )
         seaLevelPressure = tempValue;
      Serial.printf( "seaLevelPressure has been changed to %f.\n", seaLevelPressure );
   }
   else if( strcmp( command, "deviceRestart" ) == 0 )
   {
      Serial.printf( "Restarting this device...\n" );
      deviceRestart();
   }
   else
      Serial.printf( "Unknown command '%s'\n", command );
} // End of the mqttCallback() function.
