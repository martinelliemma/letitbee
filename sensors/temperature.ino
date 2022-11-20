#include "DHT.h"
#define DHTPIN 4 //il pin d'ingresso del microcontrollore dove e' connesso il sensore
#define DHTTYPE DHT22   // DHT 22  (AM2302), AM2321

// inizializziamo il sensore
DHT dht(DHTPIN, DHTTYPE);
void setup() {
  Serial.begin(9600);
  dht.begin();
}

void loop() {
  // ad ogni lettura introduciamo un ritardo di due secondi
  delay(2000);
  // leggiamo l'umidita'
  float h = dht.readHumidity();
  // Leggiamo la temperatura in gradi centigradi 
  float t = dht.readTemperature();
  
  // controlliamo se abbiamo letto correttamente i dati .
  if (isnan(h) || isnan(t) ) {
    Serial.println(" La lettura e' fallita");
    return;
  }
  
  Serial.print("Umidita': ");
  Serial.print(h);
  Serial.print("%  Temperatura: ");
  Serial.print(t);
  Serial.println("");
  
}
