#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <UniversalTelegramBot.h>       //libreiria bot scritta da Brian Lough: https://github.com/witnessmenow/Universal-Arduino-Telegram-Bot
#include <ArduinoJson.h>
#include <HX711_ADC.h>
#include <EEPROM.h>
#include <Servo_ESP32.h>
#include <Pangodream_18650_CL.h>
#include <WebSerial.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <arduinoFFT.h>

// sensore di temperatura
#include "DHT.h"
#define DHTPIN 32                  // il pin d'ingresso del microcontrollore dove e' connesso il sensore
#define DHTTYPE DHT22             // DHT 22  (AM2302), AM2321
DHT dht(DHTPIN, DHTTYPE);         // indichiamo il pin dov e è connesso il sensore e la tipologia

// bilancia
//pins:
#define HX711_dout 25 //mcu > HX711 dout pin
#define HX711_sck 26 //mcu > HX711 sck pin
//HX711 constructor:
HX711_ADC LoadCell(HX711_dout, HX711_sck);
const int calVal_calVal_eepromAdress = 0;

//frequenza
arduinoFFT FFT = arduinoFFT();
#define CHANNEL 33
const uint16_t samples = 256; //This value MUST ALWAYS be a power of 2
const double samplingFrequency = 10000; //Hz, must be less than 10000 due to ADC //Cambia in base alla ferquenza massima nello spettro
unsigned int sampling_period_us;
unsigned long microseconds;
double vReal[samples];
double vImag[samples];
#define SCL_INDEX 0x00
#define SCL_TIME 0x01
#define SCL_FREQUENCY 0x02
#define SCL_PLOT 0x03

//servomotore
static const int servoPin = 14; //printed G14 on the board
Servo_ESP32 servo1;
int angle =0;
int angleStep = 5;
int angleMin =0;
int angleMax = 180;

//batteria
Pangodream_18650_CL BL;

// Indica le credenziali del tuo WIFI
const char* ssid = "Myhotspot";        //               //"FASTWEB-57PD62"; 
const char* password = "2357111317";       //             //"N3MJKFLRPJ";

AsyncWebServer server(80);

// Inizializzazione bot 
#define BOTtoken "5650018451:AAFsFVeiwHHpDRPd_KsXtrDixpGaWMjF3jY"  // inserisci il tuo toke, che hai ricevuto da Botfather

//Inseriamo la chat id cbcla procedura è stata indicata nel post 
#define CHAT_ID "983585668"
WiFiClientSecure client;
UniversalTelegramBot bot(BOTtoken, client);

// E' ritardo con il quale la nostra applicazione interroga il server di Telegram
int botRequestDelay = 1000;
unsigned long lastTimeBotRan;

float h, t, p, f, f_mean, b;
String ts, hs, ps, f_s, bs;

// Stampiamo sul terminale della seriale indicazioni sul messaggio ricevuto
void handleNewMessages(int numNewMessages) {
  Serial.println("Gestione del messaggio");
  Serial.println(String(numNewMessages));
  for (int i=0; i<numNewMessages; i++) {
    // verifichiamo che la chat id sia corretta 
    String chat_id = String(bot.messages[i].chat_id);
    if (chat_id != CHAT_ID){
      bot.sendMessage(chat_id, "Utente non autorizzato", "");
      continue;
    }
    
    // Stampiamo sul bot l'elenco dei comandi
    String text = bot.messages[i].text;
    Serial.println(text);
    String from_name = bot.messages[i].from_name;
    if (text == "/start") {
      String welcome = "Welcome, " + from_name + ".\n";
      welcome += "Use the following commands to control your outputs.\n\n";
      welcome += "/temperature temperature \n";
      welcome += "/humidity humidity \n";
      welcome += "/weight weight \n";
      welcome += "/frequency frequency \n";
      welcome += "/battery battery \n";
      welcome += "/all temperature, humidity, weight, frequency, battery \n";
      bot.sendMessage(chat_id, welcome, "");
    }
    if (text == "/temperature") {
      ts = String(t);
      bot.sendMessage(chat_id, ts, ""); 
    }else if (text == "/humidity") {
      hs = String(h);
      bot.sendMessage(chat_id, hs, ""); 
    }else if (text == "/weight") {
      ps = String(p);
      bot.sendMessage(chat_id, ps, ""); 
    }else if (text == "/frequency") {
      f_s = String(f);
      bot.sendMessage(chat_id, f_s, ""); 
    }else if (text == "/battery") {
      bs = String(b);
      bot.sendMessage(chat_id, bs, ""); 
    }else if (text == "/all") {
      ts = String(t);
      hs = String(h);
      ps = String(p);
      f_s = String(f);
      bs = String(b);
      bot.sendMessage(chat_id, ts, "");
      bot.sendMessage(chat_id, hs, ""); 
      bot.sendMessage(chat_id, ps, ""); 
      bot.sendMessage(chat_id, f_s, ""); 
      bot.sendMessage(chat_id, bs, "");
    }
  }
}


void PrintVector(double *vData, uint16_t bufferSize, uint8_t scaleType) {
for (uint16_t i = 0; i < bufferSize; i++){
    double abscissa;
    /* Print abscissa value */ //abscissa value is the band of frequencies shown
    switch (scaleType)
    {
      case SCL_INDEX:
        abscissa = (i * 1.0);
  break;
      case SCL_TIME:
        abscissa = ((i * 1.0) / samplingFrequency);  //(1 / samplingFrequency) = tempo tra un campionamento e l'altro
  break;
      case SCL_FREQUENCY:
        abscissa = ((i * 1.0 * samplingFrequency) / samples);   //kf (Video: https://www.youtube.com/watch?v=dCeHOf4cJE0)
  break;
    }
    Serial.print(",");  
    Serial.print(abscissa, 6);  //Cosa vuol dire Serial.print(variabile da stampare, numero?);
    Serial.print(","); 
    Serial.println(vData[i], 4);
  }
  yield();
  delay(100); 
}

void MeanMagn(double *vData) {
  int meanmagn=0;
  int meanmagnD=0;
  
  for (uint16_t i = 2; i < 13; i++) {
    double abscissa;
     abscissa = ((i * 1.0 * samplingFrequency) / samples); 
     meanmagn=meanmagn+(abscissa*vData[i]);
     meanmagnD=meanmagnD+vData[i];
  }
  f_mean=meanmagn/meanmagnD;
  Serial.print(f_mean); 
  Serial.print(","); 

  yield();
  delay(100); 
}


void frequenza (){
  microseconds = micros();   //Guarda come funziona
  for(int i=0; i<samples; i++) {
      vReal[i] = analogRead(CHANNEL);
      vImag[i] = 0;
      while(micros() - microseconds < sampling_period_us){ //rallenta e blocca lo scketch fino a quando non e' passato il tempo di campionamento
        //empty loop
      }
      microseconds += sampling_period_us;
  }

  /* Print the results of the sampling according to time */
  Serial.println("Data:");
  PrintVector(vReal, samples, SCL_TIME); 
  FFT.Windowing(vReal, samples, FFT_WIN_TYP_HAMMING, FFT_FORWARD); 
  FFT.Compute(vReal, vImag, samples, FFT_FORWARD);
  FFT.ComplexToMagnitude(vReal, vImag, samples); /* Compute magnitudes */  //Vengono dati i valori reali ed immaginari vengono poi dati ad FFT.ComplexToMagnitude che restituisce le ampiezze in funzione della ferquenza
  Serial.println("Computed magnitudes:");
  PrintVector(vReal, (samples >> 1), SCL_FREQUENCY);  
  
  f = FFT.MajorPeak(vReal, samples, samplingFrequency);  //Trova la frequenza piu' importante quidni forse è meglio usare questa
  Serial.print(",");  
  Serial.println(f, 6); //Print out what frequency is the most dominant. Calcola risoluzione frequenze dal video (https://www.youtube.com/watch?v=dCeHOf4cJE0)
  Serial.print(",");
  MeanMagn(vReal);    //frequenza da usare per if
}


void temperatura (){   
  h = dht.readHumidity();
  t = dht.readTemperature();
    
  Serial.print(t);
  Serial.print(",");
  Serial.print(h);
  Serial.print(",");
  if (t>340){
      bot.sendMessage("Temperature is increasing swarming event is closer", "");
      for(int angle = 0; angle <= angleMax; angle +=angleStep) {
        servo1.write(angle);
        Serial.println(angle);
        delay(20);
      }
  }
}

void bilancia (){
  static boolean newDataReady = 0;
  const int serialPrintInterval = 2000; //increase value to slow down serial print activity

  // check for new data/start next conversion:
  if (LoadCell.update()) newDataReady = true;

  // get smoothed value from the dataset:
  if (newDataReady) {
    p = LoadCell.getData();
    Serial.print(p);
    Serial.print(",");
    newDataReady = 0;
  }
}

void batteria (){
//  Serial.print("Value from pin: ");
//  Serial.println(analogRead(34));
//  Serial.print("Average value from pin: ");
//  Serial.println(BL.pinRead());
//  Serial.print("Volts: ");
//  Serial.println(BL.getBatteryVolts());
//  Serial.print("Charge level: ");
//  Serial.println(BL.getBatteryChargeLevel());
//  Serial.println("");
  b=BL.getBatteryChargeLevel();
  if (b<15){
      bot.sendMessage("Battery is at 15%", "");
  }
}

void setup() {
  Serial.begin(115200);
  dht.begin();
  servo1.attach(servoPin);
  
  // Connessione al WIFI
  // sarebbe meglio metterlo nel loop così non rimane connesso e usa meno batteria
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  client.setCACert(TELEGRAM_CERTIFICATE_ROOT); // aggiungiamo il  certificato root per api.telegram.org
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("connessione al WI-FI..");
  }
  // Stampiamo sul terminale della seriale l'IP locale dell'ESP32
  Serial.println(WiFi.localIP());

  //frequenza
  sampling_period_us = round(1000000*(1.0/samplingFrequency));

  //bilancia
  float calibrationValue; // calibration value
  calibrationValue = 34.34; // uncomment this if you want to set this value in the sketch
  EEPROM.begin(512); // uncomment this if you use ESP8266 and want to fetch this value from eeprom

  LoadCell.begin();
  unsigned long stabilizingtime = 2000; // tare preciscion can be improved by adding a few seconds of stabilizing time
  boolean _tare = true; //set this to false if you don't want tare to be performed in the next step
  LoadCell.start(stabilizingtime, _tare);
  if (LoadCell.getTareTimeoutFlag()) {
    Serial.println("Timeout, check MCU>HX711 wiring and pin designations");
  }
  else {
    LoadCell.setCalFactor(calibrationValue); // set calibration factor (float)
    Serial.println("Startup is complete");
  }
  
  if (LoadCell.getSPS() < 7) {
    Serial.println("!!Sampling rate is lower than specification, check MCU>HX711 wiring and pin designations");
  }
  else if (LoadCell.getSPS() > 100) {
    Serial.println("!!Sampling rate is higher than specification, check MCU>HX711 wiring and pin designations");
  }
}

void loop() {
  if (millis() > lastTimeBotRan + botRequestDelay)  {
    int numNewMessages = bot.getUpdates(bot.last_message_received + 1);

    frequenza();

    temperatura();

    bilancia();

    batteria();

    while(numNewMessages) {
      Serial.println("got response");
      handleNewMessages(numNewMessages);
      numNewMessages = bot.getUpdates(bot.last_message_received + 1);
  }
  lastTimeBotRan = millis();
  }
  
  delay(1000*58);       //leggiamo i valori ogni minuto
}
