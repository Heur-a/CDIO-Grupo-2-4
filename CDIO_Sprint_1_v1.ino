#define power_pin 5
#include <math.h>
#include <Wire.h>
#include <Adafruit_ADS1X15.h>

Adafruit_ADS1115 ads1115; // construct an ads1115 at address 0x48
const int AirValue = 30150;  // Medimos valor en seco
const int WaterValue = 17300;  // Medimos valor en agua
int counter = 0;
int rep = 5;


void setup () {
  pinMode(power_pin, OUTPUT);
  Serial.begin(9600);
  Serial.println("Inicializando...");
  ads1115.begin(0x48); //Initialize ads1115
  Serial.println("Ajustando la ganancia...");
  ads1115.setGain(GAIN_ONE);

  Serial.println("Tomando medidas del canal AIN0");

  Serial.println("Rango del ADC: +/- 4.096V (1 bit=2mV)");
}

void loop() {
  int16_t adc0;

  int16_t adc1;
  int16_t humedad;
  adc1 = ads1115.readADC_SingleEnded(0);
  humedad = 100 * AirValue / (AirValue - WaterValue) - adc1 * 100 / (AirValue - WaterValue);


  /*Serial.print("AIN0: ");
    Serial.println(adc1);
    Serial.print("HR (%): ");
    Serial.print(humedad);
    Serial.println("%");*/



  //Alimentamos sonda con tren de pulsos
  digitalWrite(power_pin, HIGH);
  delay (100);

  //leemos cuando hay nivel alto
  adc0 = analogRead(A0);
  digitalWrite(power_pin, LOW);
  delay(100);
  if (adc0 < 573) {
    adc0 = 573;
  }
  float ValSal = 0;
  ValSal = 0.0000007 * adc0 * adc0 * adc0 - 0.0006 * adc0 * adc0 + 0.114 * adc0;

  //Imprimimos info
  if (counter % rep == 0) {
    //Serial.print("Lectura digital sal = "); Serial.println(adc0, DEC);
    if (adc0  == 573) {
      ValSal = 0;
    }
    Serial.print("Cantidad de sal g= "); Serial.println(ValSal, DEC);
  }


  counter = counter + 1;
}
