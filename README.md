cambiare nome al fft_03 in microfono
# :honeybee: LET IT BEE🍯 
Digital system design project

This is a university project whose aim is to check if a swarm event is happening using an esp32 board, sound and temperature sensore, a weight scale, a gate to stop the queen from swarming end a gate that open up when the queen want to swarm in order to let the bee go in the honeybag. It also send the user using telegram bot a notification of the swarming event and also the user can comunicate with the hive. There is also a file that can be used to monitor the hive all year: there is the cheacking on the production of honey and if during winter the hive have eaten all their food. The code is in continue evolution so check in time, it could change.

## 📌 Feature
- [x] Temperature sensor reading 
- [x] Sound sensor reading
- [x] Weight scale sensor reading
- [x] Grid to stop the queen from swarning
- [x] Open escape hole for swarming inside the honeybag
- [x] Wi-fi connection
- [x] Telegram interfacing
- [x] Programmable events with Telegram and monitoring
- [x] Viewing of the data on Webserial

## 💾 Circuit
![sketch of the wire connection](https://user-images.githubusercontent.com/118644154/207912455-bfd0ee72-380d-4163-9548-bec6b762368d.jpg)

## 📚 Used Libraries
* WiFi.h
* WiFiClientSecure.h
* UniversalTelegramBot.h       libreiria bot written by Brian Lough: https://github.com/witnessmenow/Universal-Arduino-Telegram-Bot
* ArduinoJson.h
* HX711_ADC.h
* EEPROM.h
* Servo_ESP32.h
* Pangodream_18650_CL.h
* WebSerial.h
* ESPAsyncWebServer.h
* arduinoFFT.h
* NTPClient.h
* WiFiUdp.h
* DHT.h

## 3D GRID
