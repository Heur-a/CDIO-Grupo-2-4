#include <Adafruit_ADS1X15.h>

#define channelvalue 0
#define Offset 0.00
#define samplingInterval 20
#define printInterval 20
#define printInterval 800
#define ArrayLength 40
int pHArray[ArrayLength];
int pHArrayIndex=0;

//Construimos el ADS1115
Adafruit_ADS1X15 ads1115; 

void setup() {
  Serial.begin(9600);
  Serial.println("Inicializamos el medidor de PH");
  ads1115.begin(0x48);
  ads1115.setGain(GAIN_ONE);
}

void loop(void) {
  static unsigned long samplingTime=millis();
  static unsigned long printTime=millis();
  static float pHValue, voltage;
  if(millis() - samplingTime > samplingInterval){
    pHArray[pHArrayIndex++]= ads1115.readADC_SingleEnded(0);
    if(pHArrayIndex== ArrayLength)pHArrayIndex = 0;

    //Convertir la lectura en tension
    voltage=ads1115.readADC_SingleEnded(0);
    pHValue=3.5*voltage+Offset;
    samplingTime=millis();
  }
  if(millis()- printTime>printInterval){
    Serial.print("Voltage:");
    Serial.print(voltage,2);
    Serial.print("  pH value: ");
    Serial.println(pHValue,2);
    printTime=millis();
  }
}
