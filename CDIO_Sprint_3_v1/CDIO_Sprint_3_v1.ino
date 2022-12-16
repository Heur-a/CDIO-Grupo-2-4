#define power_pin 5
#include <math.h>
#include <Wire.h>
#include <Adafruit_ADS1X15.h>
//pH defines
#define channelvalue 0
#define Offset 0.00
#define samplingInterval 20
#define printInterval 20
#define printInterval 800
#define ArrayLength 40 
//------------------------------------------------------------------------------------
//
//              CANALS SENSORS
//
//------------------------------------------------------------------------------------
#define canalpH 3
#define canalLlum 1
#define canalTemp  0
#define canalHum 2
#define canalSal A0

//----------------------------------------------------------------------------------
//
//                  INIT ADAFRUIT I HUMITAT
//
//-----------------------------------------------------------------------------

Adafruit_ADS1115 ads1115; // construct an ads1115 at address 0x48
const int AirValue = 30150;  // Medimos valor en seco
const int WaterValue = 17300;  // Medimos valor en agua
int counter = 0;
int rep = 5;

//--------------------------------------------------------------------------------
//
//                    SETUP
//
//---------------------------------------------------------------------------------
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
/*-----------------------------------------------------------------------------------------------
|
|
|
|                          canaladc -> FUNCIONS() -> valor,float
|
|                        Magnituds amb funció:
|                          LLUM
|                          HUM*
|                          SAL
|                          pH
|                          TEMP*
|
|                    *per implementar correctament
|
|                               | | | | | | | | | | | | |
|                               v v v v v v v v v v v v v
------------------------------------------------------------------------------------------------*/


//-----------------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------
//            FUNC LLUM
//------------------------------------------------------
//-----------------------------------------

float funcLlum(unsigned int canalAdc, int16_t * adcOut ){
  int16_t adc;
  adc = ads1115.readADC_SingleEnded(canalAdc);
  (*adcOut) = adc;
  static float Vout;
  Vout = ((4.096) * adc) / (32767); //formula Luminositat
  return Vout;

}

//-----------------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------
//            FUNC TEMPERAURA
//------------------------------------------------------
//-----------------------------------------


float funcTemp(unsigned int canalAdc){
  
  int16_t adc0 = ads1115.readADC_SingleEnded(0);

  Serial.println(adc0);

  float temperatura = ((((adc0*4.096)/32767)-0.75)/0.037)-5;
  //  float temperatura = ((((adc0*4.096)/32767)-0.79)/0.035)-5;


  Serial.print("La temperatura es: ");
  Serial.print(temperatura);
  Serial.println(" ºC");
  delay(1000);
}

//-----------------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------
//            FUNC HUMEDAD
//------------------------------------------------------
//-----------------------------------------

float  funcHum(unsigned int canalAdc ) {


  int16_t adc1;
  /*
  int16_t humedad;
  adc1 = ads1115.readADC_SingleEnded(0);
  humedad = 100 * AirValue / (AirValue - WaterValue) - adc1 * 100 / (AirValue - WaterValue);


  /*Serial.print("AIN0: ");
    Serial.println(adc1);
    Serial.print("HR (%): ");
    Serial.print(humedad);
    Serial.println("%");*/
     //return humedad;*/
     
  int  sensorValue = analogRead (canalAdc);
  int humidityValue = 100 * AirValue / (AirValue - WaterValue) - sensorValue * 100 / (AirValue - WaterValue);
  Serial.println ();
  Serial.print ("Humedad: ");
  Serial.print (humidityValue);
  Serial.println("%");
  delay (1000);

}
  //---------------------------------------------------------------
  //-------------------------------------------------------
  //      FUNCION SAL
  //------------------------------------------------
  //-----------------------------------------
  
float funcSal(uint8_t canalAdc) {
  int16_t adc0;
  //Alimentamos sonda con tren de pulsos
  digitalWrite(power_pin, HIGH);
  delay (100);

  //leemos cuando hay nivel alto
  adc0 = analogRead(canalAdc);
  digitalWrite(power_pin, LOW);
  delay(100);
  if (adc0 < 573) {
    adc0 = 573;  //Ocultem datos xicotets que donarien resultats incogruents
  }
  float ValSal = 0;
  ValSal = 0.0000007  * pow(adc0,3) - 0.0006 *pow(adc0,2) + 0.114 * adc0; //Lagrange fórmula
  if (adc0  == 573) {
    ValSal = 0; //si les dades son del rango on no dona resultats son 0g, que deuria de ser del rang
  }             // de 0 < x < 5 g

  return ValSal;
  //Imprimimos info
  if (counter % rep == 0) {
    //Serial.print("Lectura digital sal = "); Serial.println(adc0, DEC);
    Serial.print("Cantidad de sal g= "); Serial.println(ValSal, DEC);
  }


  counter = counter + 1;
}

//-------------------------------------------------------------------------------
//--------------------------------------------------------------------
//    FUNCIO PH
//------------------------------------------------------
//---------------------------------------------


float funcpH (int canalAdc) {

  int pHArray[ArrayLength];
  int pHArrayIndex = 0;
  static unsigned long samplingTime = millis();
  static unsigned long printTime = millis();
  static float pHValue, voltage;
  if (millis() - samplingTime > samplingInterval) {
    pHArray[pHArrayIndex++] = ads1115.readADC_SingleEnded(canalAdc);
    if (pHArrayIndex == ArrayLength){
      pHArrayIndex = 0;
      double counter = 0.0;
      for (int i = 0; i < ArrayLength; i++){
        counter = pHArray[i] + counter; 
      } 
    }
    double valueIpH = counter/ArrayLength;
    //Convertir la lectura en tension


    voltage = (4.096 / 32767.0) * valueIpH;
    pHValue = 3.5 * voltage + Offset;
    samplingTime = millis();
    return pHValue;
  }
  


}
void loop() {
  float pH,VoltLlum,Temp,Hum,Sal,voltatgepH = 0;
  int16_t adcLlum = 0;
  pH = funcpH(canalpH);
  VoltLlum = funcLlum(canalLlum, &adcLlum);
  Sal = funcSal(canalSal);
  Temp = funcTemp (canalTemp);
  // Hum = funcHum(canalHum)

//--------------PRINT----------------------
static unsigned long printTime = millis();
 if (millis() - printTime > printInterval) {
    //pH
    Serial.print("  pH value: ");
    Serial.println(pH, 2);
    printTime = millis();
    
     //Temperatura
    Serial.print("La temperatura es: "); Serial.println(Temp,2);
     //Sal

    Serial.print("Cantidad de sal g= "); Serial.println(Sal,2);
    //Llum

    Serial.print("V out Llum:"); Serial.print(VoltLlum, 3); Serial.println ("V");
    if (adcLlum <= 200) {
      Serial.println("Esta a la sombra");
    }

    if (adcLlum > 200 && adcLlum <= 1000) {
      Serial.println("Luz natural ambiente");
    }
    if (adcLlum > 1000) {
      Serial.println("Luz extrema");
    }



    printTime = millis();

  }






   

  }
