#define power_pin 5
#include <math.h>
#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include <ESP8266WiFi.h>
//pH defines
#define channelvalue 0
#define Offset 0.00
#define samplingInterval 20
#define printInterval 800
#define ArrayLength 40 



// Comentar/Descomentar para ver mensajes de depuracion en monitor serie y/o respuesta del HTTP server
//#define PRINT_DEBUG_MESSAGES
//#define PRINT_HTTP_RESPONSE

// Comentar/Descomentar para conexion Fuera/Dentro de UPV
//#define WiFi_CONNECTION_UPV

// Selecciona que servidor REST quieres utilizar entre ThingSpeak y Dweet
#define REST_SERVER_THINGSPEAK //Selecciona tu canal para ver los datos en la web (https://thingspeak.com/channels/360979)
///////////////////////////////////////////////////////
/////////////// WiFi Definitions /////////////////////
//////////////////////////////////////////////////////

#ifdef WiFi_CONNECTION_UPV //Conexion UPV
  const char WiFiSSID[] = "GTI1";
  const char WiFiPSK[] = "1PV.arduino.Toledo";
#else //Conexion fuera de la UPV
  const char WiFiSSID[] = "Redmi";
  const char WiFiPSK[] = "19467382";
#endif



///////////////////////////////////////////////////////
/////////////// SERVER Definitions /////////////////////
//////////////////////////////////////////////////////

#if defined(WiFi_CONNECTION_UPV) //Conexion UPV
  const char Server_Host[] = "proxy.upv.es";
  const int Server_HttpPort = 8080;
#elif defined(REST_SERVER_THINGSPEAK) //Conexion fuera de la UPV
  const char Server_Host[] = "api.thingspeak.com";
  const int Server_HttpPort = 80;
#else
  const char Server_Host[] = "dweet.io";
  const int Server_HttpPort = 80;
#endif

WiFiClient client;

///////////////////////////////////////////////////////
/////////////// HTTP REST Connection ////////////////
//////////////////////////////////////////////////////

#ifdef REST_SERVER_THINGSPEAK 
  const char Rest_Host[] = "api.thingspeak.com";
  String MyWriteAPIKey="J2M8VDF98NY5Z1JZ"; // Escribe la clave de tu canal ThingSpeak
#else 
  const char Rest_Host[] = "api.thingspeak.com";
  String MyWriteAPIKey="J2M8VDF98NY5Z1JZ"; // Escribe la clave de tu canal Dweet
#endif

#define NUM_FIELDS_TO_SEND 5 //Numero de medidas a enviar al servidor REST (Entre 1 y 8)

/////////////////////////////////////////////////////
/////////////// Pin Definitions ////////////////
//////////////////////////////////////////////////////

const int LED_PIN = 4; // Thing's onboard, green LED

/////////////////////////////////////////////////////
/////////////// WiFi Connection ////////////////
//////////////////////////////////////////////////////

void connectWiFi()
{
  byte ledStatus = LOW;

  #ifdef PRINT_DEBUG_MESSAGES
    Serial.print("MAC: ");
    Serial.println(WiFi.macAddress());
  #endif
  
  WiFi.begin(WiFiSSID, WiFiPSK);

  while (WiFi.status() != WL_CONNECTED)
  {
    // Blink the LED
    digitalWrite(LED_PIN, ledStatus); // Write LED high/low
    ledStatus = (ledStatus == HIGH) ? LOW : HIGH;
    #ifdef PRINT_DEBUG_MESSAGES
       Serial.println(".");
    #endif
    delay(500);
  }
  #ifdef PRINT_DEBUG_MESSAGES
     Serial.println( "WiFi Connected" );
     Serial.println(WiFi.localIP()); // Print the IP address
  #endif
}

/////////////////////////////////////////////////////
/////////////// HTTP POST  ThingSpeak////////////////
//////////////////////////////////////////////////////

void HTTPPost(String fieldData[], int numFields){

// Esta funcion construye el string de datos a enviar a ThingSpeak mediante el metodo HTTP POST
// La funcion envia "numFields" datos, del array fieldData.
// Asegurate de ajustar numFields al número adecuado de datos que necesitas enviar y activa los campos en tu canal web
  
    if (client.connect( Server_Host , Server_HttpPort )){
       
        // Construimos el string de datos. Si tienes multiples campos asegurate de no pasarte de 1440 caracteres
   
        String PostData= "api_key=" + MyWriteAPIKey ;
        for ( int field = 1; field < (numFields + 1); field++ ){
            PostData += "&field" + String( field ) + "=" + fieldData[ field ];
        }     
        
        // POST data via HTTP
        #ifdef PRINT_DEBUG_MESSAGES
            Serial.println( "Connecting to ThingSpeak for update..." );
        #endif
        client.println( "POST http://" + String(Rest_Host) + "/update HTTP/1.1" );
        client.println( "Host: " + String(Rest_Host) );
        client.println( "Connection: close" );
        client.println( "Content-Type: application/x-www-form-urlencoded" );
        client.println( "Content-Length: " + String( PostData.length() ) );
        client.println();
        client.println( PostData );
        #ifdef PRINT_DEBUG_MESSAGES
            Serial.println( PostData );
            Serial.println();
            //Para ver la respuesta del servidor
            #ifdef PRINT_HTTP_RESPONSE
              delay(500);
              Serial.println();
              while(client.available()){String line = client.readStringUntil('\r');Serial.print(line); }
              Serial.println();
              Serial.println();
            #endif
        #endif
    }
}

////////////////////////////////////////////////////
/////////////// HTTP GET  ////////////////
//////////////////////////////////////////////////////

void HTTPGet(String fieldData[], int numFields){
  
// Esta funcion construye el string de datos a enviar a ThingSpeak o Dweet mediante el metodo HTTP GET
// La funcion envia "numFields" datos, del array fieldData.
// Asegurate de ajustar "numFields" al número adecuado de datos que necesitas enviar y activa los campos en tu canal web
  
    if (client.connect( Server_Host , Server_HttpPort )){
           #ifdef REST_SERVER_THINGSPEAK 
              String PostData= "GET https://api.thingspeak.com/update?api_key=";
              PostData= PostData + MyWriteAPIKey ;
           #else 
              String PostData= "GET http://dweet.io/dweet/for/";
              PostData= PostData + MyWriteAPIKey +"?" ;
           #endif
           
           for ( int field = 1; field < (numFields + 1); field++ ){
              PostData += "&field" + String( field ) + "=" + fieldData[ field ];
           }
          
           
           #ifdef PRINT_DEBUG_MESSAGES
              Serial.println( "Connecting to Server for update..." );
           #endif
           client.print(PostData);         
           client.println(" HTTP/1.1");
           client.println("Host: " + String(Rest_Host)); 
           client.println("Connection: close");
           client.println();
           #ifdef PRINT_DEBUG_MESSAGES
              Serial.println( PostData );
              Serial.println();
              //Para ver la respuesta del servidor
              #ifdef PRINT_HTTP_RESPONSE
                delay(500);
                Serial.println();
                while(client.available()){String line = client.readStringUntil('\r');Serial.print(line); }
                Serial.println();
                Serial.println();
              #endif
           #endif  
    }
}











//------------------------------------------------------------------------------------
//
//              CANALS SENSORS
//
//------------------------------------------------------------------------------------
#define canalpH 2
#define canalLlum 1
#define canalTemp  0
#define canalHum 3

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
   #ifdef PRINT_DEBUG_MESSAGES
    Serial.begin(9600);
  #endif
  
  connectWiFi();
  digitalWrite(LED_PIN, HIGH);
  Serial.begin(9600);

  #ifdef PRINT_DEBUG_MESSAGES
      Serial.print("Server_Host: ");
      Serial.println(Server_Host);
      Serial.print("Port: ");
      Serial.println(String( Server_HttpPort ));
      Serial.print("Server_Rest: ");
      Serial.println(Rest_Host);
  #endif
  //----------------------------------------------------
  //----------------------------------------------------


  pinMode(power_pin, OUTPUT);
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
|                          TEMP
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

int16_t funcLlum(unsigned int canalAdc, float * V ){
  int16_t adc;
  adc = ads1115.readADC_SingleEnded(canalAdc);
   static float Vout;
  Vout = ((4.096) * adc) / (32767); //formula Luminositat
  (*V) = Vout;
  return adc;
 

}

//-----------------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------
//            FUNC TEMPERAURA
//------------------------------------------------------
//-----------------------------------------


float funcTemp(unsigned int canalAdc){
  
 int16_t adc0 = ads1115.readADC_SingleEnded(0);

  Serial.println(adc0);

  float temperatura = ((((adc0*4.096)/32767)-0.75)/0.037);
  //float temperatura = ((adc0*3.3)-(32767*b))/(32767.0*m);

  return temperatura + 40;
}
//-----------------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------
//            FUNC HUMEDAD
//------------------------------------------------------
//-----------------------------------------

float  funcHum(unsigned int canalAdc ) {
int16_t adc1;
int16_t humedad;
  
int ValorSec = 30000;
int ValorHumit = 17000;
adc1 = ads1115.readADC_SingleEnded(canalAdc);
humedad = map(adc1, ValorSec,ValorHumit,0,100);
  if(humedad > 100){
    humedad = 100;
  }
return humedad;

  
  
}
  //---------------------------------------------------------------
  //-------------------------------------------------------
  //      FUNCION SAL
  //------------------------------------------------
  //-----------------------------------------
  
float funcSal(float *VoltSal) {
  int16_t adc0;
  //Alimentamos sonda con tren de pulsos
  digitalWrite(power_pin, HIGH);
  delay (100);

  //leemos cuando hay nivel alto
  adc0 = analogRead(A0);
  digitalWrite(power_pin, LOW);
  delay(100);
  (*VoltSal) = adc0;
  if (adc0 < 573) {
    adc0 = 573;  //Ocultem datos xicotets que donarien resultats incogruents
  }
  float ValSal = 0;
  ValSal = 0.0000007  * pow(adc0,3) - 0.0006 *pow(adc0,2) + 0.114 * adc0; //Lagrange fórmula
  if (adc0  == 573) {
    ValSal = 0; //si les dades son del rango on no dona resultats son 0g, que deuria de ser del rang
  }             // de 0 < x < 5 g

  return ValSal;

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
  //static unsigned long printTime = millis();
  static float pHValue, voltage;
  if (millis() - samplingTime > samplingInterval) {
   /* pHArray[pHArrayIndex++] = ads1115.readADC_SingleEnded(canalAdc);
    if (pHArrayIndex == ArrayLength){
      pHArrayIndex = 0;
      double counter = 0.0;
      for (int i = 0; i < ArrayLength; i++){
        counter = pHArray[i] + counter; 
      } 
    }
    */
    double valueIpH = ads1115.readADC_SingleEnded(canalAdc);
    //Convertir la lectura en tension


    voltage = (4.096 / 32767.0) * valueIpH;
    pHValue = 3.5 * voltage + Offset;
    samplingTime = millis();
    ;
  }
  return pHValue;
}//()
//-----------------------------------------------------------
//-----------------------------------------------------------
//
//            LOOP
//
//-----------------------------------------------------------
//-----------------------------------------------------------
  



void loop() {
  float pH,VoltLlum,Temp,Hum,Sal,voltatgepH = 0;
  float VoltSal;
  int16_t adcLlum = funcLlum(canalLlum,&VoltLlum);
  pH = funcpH(canalpH);
  Sal = funcSal( &VoltSal);
  Temp = funcTemp (canalTemp);
  Hum = funcHum(canalHum)

//--------------PRINT----------------------
static unsigned long printTime = millis();
 if (millis() - printTime > printInterval) {
    //pH
    Serial.print("  pH value: ");
    Serial.println(pH, 2);
    printTime = millis();
    //Humedad
    Serial.print(" Humedad ");
    Serial.print(Hum);
    Serial.println("%");
    
     //Temperatura
    Serial.print("La temperatura es: "); Serial.println(Temp,2);
     //Sal

    Serial.print("Cantidad de sal g= "); Serial.println(Sal,2);
    Serial.print ("VoltSal = ");  Serial.println(VoltSal,DEC);
    //Llum

    Serial.print("V out Llum:"); Serial.print(VoltLlum, 3); Serial.println ("V");
    Serial.print("adcLlum"); Serial.println(adcLlum);
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

    String data[ NUM_FIELDS_TO_SEND + 1];  // Podemos enviar hasta 8 datos

    
    data[ 1 ] = String( pH); //Escribimos el dato 1. Recuerda actualizar numFields
    #ifdef PRINT_DEBUG_MESSAGES
        Serial.print( "pH = " );
        Serial.println( data[ 1 ] );
    #endif

    data[ 2 ] = String(Sal ); //Escribimos el dato 2. Recuerda actualizar numFields
    #ifdef PRINT_DEBUG_MESSAGES
        Serial.print( "Sal = " );
        Serial.println( data[ 2 ] );
    #endif
    data[ 3 ] = String(VoltLlum ); //Escribimos el dato 2. Recuerda actualizar numFields
    #ifdef PRINT_DEBUG_MESSAGES
        Serial.print( "V LLum = " );
        Serial.println( data[ 3 ] );
    #endif

     data[ 4] = String(Temp ); //Escribimos el dato 2. Recuerda actualizar numFields
    #ifdef PRINT_DEBUG_MESSAGES
        Serial.print( "Temp = " );
        Serial.println( data[ 4] );
    #endif

     data[ 5 ] = String(Hum ); //Escribimos el dato 2. Recuerda actualizar numFields
    #ifdef PRINT_DEBUG_MESSAGES
        Serial.print( "HUM = " );
        Serial.println( data[ 5 ] );
    #endif
    //Selecciona si quieres enviar con GET(ThingSpeak o Dweet) o con POST(ThingSpeak)
    HTTPPost( data, NUM_FIELDS_TO_SEND );
    //HTTPGet( data, NUM_FIELDS_TO_SEND );

    //Selecciona si quieres un retardo de 15seg para hacer pruebas o dormir el SparkFun
    delay( 15000 );   
    //Serial.print( "Goodnight" );
    //ESP.deepSleep( sleepTimeSeconds * 1000000 );

  }






   

  }
