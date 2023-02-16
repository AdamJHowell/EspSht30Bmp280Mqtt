cmake_minimum_required( VERSION 3.23 )
project( EspSht30Bmp280Mqtt )

set( CMAKE_CXX_STANDARD 14 )

add_executable( EspSht30Bmp280Mqtt
		EspSht30Bmp280Mqtt.ino
		privateInfo.h
		EspSht30Bmp280Mqtt.h
		Network.ino
		Network.h )

set_target_properties( EspSht30Bmp280Mqtt PROPERTIES LINKER_LANGUAGE CXX )
