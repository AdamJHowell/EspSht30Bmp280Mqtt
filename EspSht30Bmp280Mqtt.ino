#include "EspSht30Bmp280Mqtt.h"

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
      while( true )
      {
         ArduinoOTA.handle();
         delay( 12 );
      }
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
      while( true )
      {
         ArduinoOTA.handle();
         delay( 12 );
      }
   }
   /* Default settings from datasheet. */
   bmp280.setSampling( Adafruit_BMP280::MODE_NORMAL,      /* Operating Mode. */
                       Adafruit_BMP280::SAMPLING_X2,      /* Temp. oversampling */
                       Adafruit_BMP280::SAMPLING_X16,     /* Pressure oversampling */
                       Adafruit_BMP280::FILTER_X16,       /* Filtering. */
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
 * @brief toggleLED() will change the state of the LED.
 * This function does not manage any timings.
 */
void toggleLED()
{
   if( digitalRead( ONBOARD_LED ) != LED_ON )
      digitalWrite( ONBOARD_LED, LED_ON );
   else
      digitalWrite( ONBOARD_LED, LED_OFF );
} // End of toggleLED() function.

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
   Serial.print( "  IP address: " );
   Serial.println( WiFi.localIP() );
   int wifiStatusCode = WiFi.status();
   char buffer[29];
   lookupWifiCode( wifiStatusCode, buffer );
   if( wifiStatusCode == 3 )
   {
      Serial.printf( "  IP address: %s\n", ipAddress );
      Serial.printf( "  RSSI: %ld\n", rssi );
   }
   Serial.printf( "  wifiConnectCount: %u\n", wifiConnectCount );
   Serial.printf( "  wifiCoolDownInterval: %lu\n", wifiCoolDownInterval );
   Serial.printf( "  Wi-Fi status text: %s\n", buffer );
   Serial.printf( "  Wi-Fi status code: %d\n", wifiStatusCode );
   Serial.println();

   Serial.println( "MQTT stats:" );
   Serial.printf( "  mqttConnectCount: %u\n", mqttConnectCount );
   Serial.printf( "  mqttCoolDownInterval: %lu\n", mqttCoolDownInterval );
   Serial.printf( "  Broker: %s:%d\n", mqttClient.getServerDomain(), mqttClient.getServerPort() );
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
 * @brief deviceRestart() will restart the device.
 */
void deviceRestart()
{
   Serial.println( "Restarting!" );
   ESP.restart();
} // End of the deviceRestart() function.

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

   wifiMultiConnect();
   configureOTA();

   Serial.println( "Function setup() has completed.\n" );
} // End of the setup() function.

/**
 * @brief loop() repeats over and over.
 */
void loop()
{
   // Reconnect Wi-Fi if needed, reconnect MQTT as needed.
   if( WiFi.status() != WL_CONNECTED )
      wifiMultiConnect();
   else if( !mqttClient.connected() )
      mqttConnect();

   mqttClient.loop();
   ArduinoOTA.handle();

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
      deviceRestart();
   }
} // End of the loop() function.
