#define power_pin 5
#include <math.h>
#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#define printInterval 1000
Adafruit_ADS1115 ads1115; // construct an ads1115 at address 0x48

void setup() {
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
  int16_t adc;
  adc = ads1115.readADC_SingleEnded(3);
  static float Vout;
  Vout = ((4.096) * adc) / (32767); //formula Luminositat



  static unsigned long printTime = millis();
  if (millis() - printTime > printInterval) {
    Serial.print("adc :"); Serial.println(adc);
    Serial.print("V out:"); Serial.print(Vout, 3); Serial.println ("V");
    if (adc <= 200) {
      Serial.println("Esta a la sombra");
    }

    if (adc > 200 && adc <= 1000) {
      Serial.println("Luz natural ambiente");
    }
    if (adc > 1000) {
      Serial.println("Luz extrema");
    }



    printTime = millis();

  }


}
