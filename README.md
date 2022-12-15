# 🐝LET IT BEE🍯
digital system design project
this is a university project whose aim is to chack if a swarm event is happening using an esp32 board, sound and temperature sensore, a weight scale and a gate to stop the queen from swarming. It also send the user using telegram bot a notification of the swarming event and also the user can comunicate with the hive.

la batteria non è molto accurata è meglio controllare il voltaggio
FEATURE

CIRCUIT

# Used Libraries
<WiFi.h>
#include <WiFiClientSecure.h>
#include <UniversalTelegramBot.h>       //libreiria bot scritta da Brian Lough: https://github.com/witnessmenow/Universal-Arduino-Telegram-Bot
#include <ArduinoJson.h>
#include <HX711_ADC.h>
#include <EEPROM.h>
#include <Servo_ESP32.h>
#include <Pangodream_18650_CL.h>
#include <WebSerial.h>
#include <ESPAsyncWebServer.h>
#include <arduinoFFT.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <DHT.h>

3D GRID
