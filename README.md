# Firmware Aire

## Description
'Firmware Aire' is a firmware designed to control [hw-aire](https://github.com/GuilleGonzzalez/hw-aire).
It is implemented using [Arduino Home Assistant](https://github.com/dawidchyrzynski/arduino-home-assistant/tree/main) Arduino library ([docs](https://dawidchyrzynski.github.io/arduino-home-assistant/)).

## Microcontroller
Ths firmware is designed for Espressif [ESP8266](https://www.espressif.com/en/products/modules) module, suitable for WiFi communications. 

## Entities
The device has the following entities:
 - Fan (fan)
 - Temperature sensor (temp_sensor)
 - Humidity sensor (humd_sensor)
 - Pressure sensor (press_sensor)
 - Light (general_light): for show the main air machine status.

## Features
 - Fan control using 3 relays
 - Temperature, humidity and pressure sensors
 - Buzzer
 - RGB LED for show device and main air machine status
 - User button for change fan mode (OFF/P1/P2/P3)
 - OTA update

## OTA
This firmware has Over-the-air update functionality. To perform an OTA, just indicate the IP of the device in the ```platformio.ini``` file and upload the code by pressing the ```upload``` button.