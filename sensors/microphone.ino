#include "arduinoFFT.h"

arduinoFFT FFT = arduinoFFT(); /* Create FFT object */
/*
These values can be changed in order to evaluate the functions
*/
#define CHANNEL 33
const uint16_t samples = 512; //This value MUST ALWAYS be a power of 2
const double samplingFrequency = 1800; //Hz, must be less than 10000 due to ADC //Cambia in base alla ferquenza massima nello spettro

unsigned int sampling_period_us;
unsigned long microseconds;
/*
These are the input and output vectors
Input vectors receive computed results from FFT
*/
double vReal[samples];
double vImag[samples];

//Ultima modifica
//int meanmagn=0;
//int meanmagnD=0;
//Ultima modifica

#define SCL_INDEX 0x00
#define SCL_TIME 0x01
#define SCL_FREQUENCY 0x02
#define SCL_PLOT 0x03


void setup()
{
  sampling_period_us = round(1000000*(1.0/samplingFrequency));
  Serial.begin(115200);
  Serial.println(" ");
  while(!Serial);
  //Serial.println("Ready");
}

void loop()
{
  
  /*SAMPLING*/
  microseconds = micros();   //Guarda come funziona
  for(int i=0; i<samples; i++)      //I samples indicano gli indici (equispaziati in tempo) sull'asse del tempo, non il tempo effettivo
  {
      vReal[i] = analogRead(CHANNEL);
      vImag[i] = 0;
      while(micros() - microseconds < sampling_period_us){ //rallenta e blocca lo scketch fino a quando non e' passato il tempo di campionamento
        //empty loop
      }
      microseconds += sampling_period_us;
  }

  /* Print the results of the sampling according to time */
  Serial.println("Data:");
  PrintVector(vReal, samples, SCL_TIME);   //Converte da indici in tempo


//  //Se vuoi vedere solo in funzione del tempo sul serial plotter commenta da qui fino a while(1); (INCLUSO WHILE(1) COMMENTATO)
//  //Serial.print(" , "); //Questa la avevi messa per plottare in tempo
//  //Non riesco a plottare direttamente su Arduino IDE lo spettro. Idea: Su telegram restituisco solo la ferquenza massima, poi file.txt MatLab plot piu' preciso
//  

  FFT.Windowing(vReal, samples, FFT_WIN_TYP_HAMMING, FFT_FORWARD);	/* Weigh data */ //(Video in cui spiega cosa fa il Windowing ("Prepara" i dati): https://www.youtube.com/watch?v=dCeHOf4cJE0)
  //Serial.println("Weighed data:");
  //PrintVector(vReal, samples, SCL_TIME);
  FFT.Compute(vReal, vImag, samples, FFT_FORWARD); /* Compute FFT */  //I dati "preparati" vengono dati a FFT.Compute che da come output dei valori reali ed immaginari
  //Serial.println("Computed Real values:");
  //PrintVector(vReal, samples, SCL_INDEX);
  //Serial.println("Computed Imaginary values:");
  //PrintVector(vImag, samples, SCL_INDEX);
  FFT.ComplexToMagnitude(vReal, vImag, samples); /* Compute magnitudes */  //Vengono dati i valori reali ed immaginari vengono poi dati ad FFT.ComplexToMagnitude che restituisce le ampiezze in funzione della ferquenza
  Serial.println("Computed magnitudes:");
  PrintVector(vReal, (samples >> 1), SCL_FREQUENCY);
  MeanMagn(vReal);
//  //Insert Mean in PrintVector //Ultima modifica
  
  double x = FFT.MajorPeak(vReal, samples, samplingFrequency);  //Trova la frequenza piu' importante
  Serial.print(",");  
  Serial.println(x, 6); //Print out what frequency is the most dominant. Calcola risoluzione frequenze dal video (https://www.youtube.com/watch?v=dCeHOf4cJE0)
  Serial.print(",");
  //while(1); /* Run Once */
  delay(2000); /* Repeat after delay */ //Asse del tempo per ottenere una mappa 2D come in (Identification of the honey bee swarming process by analysing the time course of hive vibrations | Elsevier Enhanced Reader)
  yield();

//  //Graph on Serial Plotter of Spetrum versus index (from number of samples) <-> frequency
//  for(int i=0;i<samples/2;i++)
//  {
//    Serial.println(vReal[i],1);
//  }
}
 

void PrintVector(double *vData, uint16_t bufferSize, uint8_t scaleType)
{
//     //Ultima modifica
//     int meanmagn=0;
//     int meanmagnD=0;
//     //Ultima modifica
  
  for (uint16_t i = 0; i < bufferSize; i++)
  {
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
    //if(scaleType==SCL_FREQUENCY)
      //Serial.print("Hz");  
      //Serial.print(",");
      //Serial.print(" ");  
      //Serial.print(",");  
      Serial.println(vData[i], 4);
      
      //Serial.println();

     //Ultima modifica
//     if(scaleType==SCL_FREQUENCY && 70<abscissa<500)
//     {
//      meanmagn=meanmagn+(abscissa*vData[i]);
//      meanmagnD=meanmagnD+vData[i];
//     }
//     yield(); 
     //Ultima modifica
  }

  //Ultima modifica
//  meanmagn=meanmagn/meanmagnD;
//  Serial.print(","); 
//  Serial.println("Mean Magnitudes");
//  Serial.println(meanmagn, 4);
//  Serial.print(","); 
  //Ultima modifica
  
  yield();
  delay(200); 
  //Serial.println();
}

void MeanMagn(double *vData)
{
  //Ultima modifica
     int meanmagn=0;
     int meanmagnD=0;
     int mean=0;
  //Ultima modifica
  
  for (uint16_t i = 7; i < 250; i++)   //(scaleType==SCL_FREQUENCY && 70<abscissa<500)
  {
    double abscissa;
     abscissa = ((i * 1.0 * samplingFrequency) / samples); 

//    Serial.print(",");  
//    Serial.print(abscissa, 6);  //Cosa vuol dire Serial.print(variabile da stampare, numero?);
//    Serial.print(",");   
//    //if(scaleType==SCL_FREQUENCY)
//      //Serial.print("Hz");  
//      //Serial.print(",");
//      //Serial.print(" ");  
//      //Serial.print(",");  
//      Serial.println(vData[i], 4);
     
     meanmagn=meanmagn+(abscissa*vData[i]);
     meanmagnD=meanmagnD+vData[i];
  }
  mean=meanmagn/meanmagnD;
  Serial.print(","); 
  Serial.println("Mean Magnitude: ");
  Serial.println(mean);  //Ultima modifica: Serial.println(mean,4); Tolto ,4 (non so cosa voglia dire, ma funziona) 
  //Serial.println(meanmagn);
  //Serial.println(meanmagnD);
  //Serial.print(","); 

  yield();
  delay(100); 
  
}
