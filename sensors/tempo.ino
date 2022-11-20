#include <NTPClient.h>
#include <WiFi.h>
#include <WiFiUdp.h>

const char *ssid     = "Myhotspot";        //               //"FASTWEB-57PD62"; 
const char* password = "2357111317";       //             //"N3MJKFLRPJ";

String dat, tim;

WiFiUDP ntpUDP;

// By default 'pool.ntp.org' is used with 60 seconds update interval and
// no offset
NTPClient timeClient(ntpUDP, "europe.pool.ntp.org", 3600, 60000);

// You can specify the time server pool and the offset, (in seconds)
// additionally you can specify the update interval (in milliseconds).
// NTPClient timeClient(ntpUDP, "europe.pool.ntp.org", 3600, 60000);
#define uS_TO_S_FACTOR 1000000
#define TIME_TO_SLEEP  10

void setup(){
  Serial.begin(115200);
  WiFi.begin(ssid, password);

  while ( WiFi.status() != WL_CONNECTED ) {
    delay ( 500 );
    Serial.print ( "." );
  }

  timeClient.begin();
  double nanna = 13.00;
  float t_double=0;
  while (t_double < nanna){
    timeClient.update();
    dat=timeClient.getFormattedDate();
    tim=timeClient.getFormattedTime();
    Serial.println(dat);
    Serial.println(tim);
    t_double=tim.toDouble();
    Serial.println(t_double);
  
    delay(1000);
  }
  
  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
  esp_deep_sleep_start();
}

void loop() {
  
}
