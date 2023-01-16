#include <Adafruit_ADS1X15.h>

#define channelvalue 0
float b=0.79;
float m=0.033;
Adafruit_ADS1X15 ads1115; 

void setup() {
  Serial.begin(9600);
  Serial.println("Inicializamos el medidor de Temperatura");
  ads1115.begin(0x48);
  ads1115.setGain(GAIN_ONE);
}

void loop(void) {
  int16_t adc0 = ads1115.readADC_SingleEnded(0);

  Serial.println(adc0);

  float vo = (adc0 * 4.096) / 32767;
  float temperatura = ((vo * 0.406) - b) / m;

  Serial.print("La temperatura es: ");
  Serial.print(temperatura);
  Serial.println(" ºC");
  delay(1000);
}
