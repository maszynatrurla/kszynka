# Kszynka - Wi-fi weather station

The least original IoT project in the world - weather station based on ESP32 uC.

## Hardware

Custom board made of following components:

 * ESP32-WROOM-32D (uC + wifi module);
 * BMP280 pressure + temperature sensor;
 * DHT-22 humidity + temperature sensor;
 * PMS7003 particle concentration sensor;
 
Later, DHT-22 was removed and BMP280 replaced with BME280.

Power is supplied by LiIon and solar batteries.
 
# Software

I didn't have much motivation, so software is mostly reused components from gnIoT project.

Application has a simple flow: 
 
 1. read data from sensors;
 2. send measurements to http server 
 3. if present - handle new configuration values from http server response
 4. go to deep sleep (for configurable number of minutes)
 

## Software Build

This software project is based on esp idf SDK. You must install it in your system, together with all dependencies
such as xtensa gcc toolchain and python modules.

Eclipse project is included. If you wish to use Eclipse IDE, you will need Eclipse with CDT plugins
 (Eclipse for C/C++ development) and cross compiler support plugin.


