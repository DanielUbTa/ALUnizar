#include <Arduino.h>
#include <SPI.h>
#include <Protocentral_ADS1220.h>
#include <7Semi_MAX31865.h>
#include "LoRa_E220.h"


int contadorPruebas = 0;

int contadorPruebas2 = 0;




#define sacarMedidasPorSerial true

//Pines SPI comunes



///////////////Globales para guardado de datos
  #define tamanoArrayFuerzas 1000
    //Equivalente a 16 segundos de guardado de datos
    //Voy a poner menos tamaño porque hay que hacer un filtro
  float arrayFuerzas[tamanoArrayFuerzas];
  uint32_t arrayFuerzasPuntero = 0;

  #define divisionRateTemperatura 100
  int divisorRateTemp = 0;
    //Cuantas veces más lento va el guardado de temperatura comparado con el de fuerza
  #define tamanoArrayTemp 256
    //Equivalente a 25 segundos de guardado de datos
  float arrayTemp[tamanoArrayTemp];
  uint32_t arrayTempPuntero = 0;

///////////////

///////////////Cosas del ADC

  #define PGA          1                 // Programmable Gain = 1
  #define VREF         2.048            // Internal reference of 2.048V
  #define VFSR         VREF/PGA
  #define FULL_SCALE   (((long int)1<<23)-1)

  const float mult = 0.000000975797*0.375;   //Esto es lo que habrá que sacar experimentalmente
  const float suma = 0.0;

  float ultimaFuerza = 0.0;
  
  #define ADC_CS   10
  #define ADC_DRDY 2
  Protocentral_ADS1220 ads1220;
  volatile bool ADCdataReady = false;


  //El ADC generará una interrupción cuando tenga un dato listo
  //Y en cada interrupción se llamará a esta función
  void IRAM_ATTR drdyISR() {
    ADCdataReady = true;
  }


  void setupADC(){

    pinMode(ADC_DRDY, INPUT);

    ads1220.begin(ADC_CS, ADC_DRDY);

    ads1220.set_pga_gain(PGA_GAIN_1); //Se le podría subir la ganancia si con los potenciometros no es suficiente
    ads1220.set_data_rate(DR_1000SPS);
    ads1220.set_conv_mode_continuous();
    ads1220.Start_Conv();

    attachInterrupt(digitalPinToInterrupt(ADC_DRDY), drdyISR, FALLING);
  }

  void guardarFuerza(){
    ads1220.Read_Data();
    float fuerzaMedida = ((float)ads1220.DataToInt()) * mult + suma;
    ultimaFuerza += 1*(fuerzaMedida-ultimaFuerza); //"Filtro"
    arrayFuerzas[arrayFuerzasPuntero] = ultimaFuerza;
    arrayFuerzasPuntero++;
    if(arrayFuerzasPuntero >= tamanoArrayFuerzas){
      arrayFuerzasPuntero = 0;
    }
  }

///////////////

///////////////Cosas del termopar
  #define RTD_CS_PIN   10
  #define RREF_OHM     430.0f
  #define R0_OHM       100.0f  //Igual hay que cambiar

  float ultimaTemp = 0.0;

  MAX31865_7Semi rtd(RTD_CS_PIN, SPI);

  void printFaultsAndClear() {
    Serial.println(F("MAX31865: FAULT detected!"));
    MAX31865_7Semi::FaultStatus f = rtd.readFaultStatus();
    if (f.rtdHigh)       Serial.println(F("  - RTD HIGH threshold"));
    if (f.rtdLow)        Serial.println(F("  - RTD LOW threshold"));
    if (f.refInHigh)     Serial.println(F("  - REFIN- > 0.85*Vbias"));
    if (f.refInLow)      Serial.println(F("  - REFIN- < 0.85*Vbias"));
    if (f.rtdInLow)      Serial.println(F("  - RTDIN- < 0.85*Vbias"));
    if (f.overUnderVolt) Serial.println(F("  - Over/Under Voltage"));
    rtd.clearFaults();
  }

  void setupRTD() {
    rtd.begin(WIRES_2,      //puede ser WIRES_2 o WIRES_3 en funcion de la sonda
              FILTER_50HZ,  // o FILTER_60HZ
              true,         // autoConvert
              true,         // Vbias ON
              1000000);     // SPI 1 MHz

    rtd.setReferenceResistor(RREF_OHM);
    rtd.setR0(R0_OHM);

    rtd.setLowThreshold(20.0f);
    rtd.setHighThreshold(40.0f);
    rtd.clearFaults();
  }

  float leerTemperaturaC() {
    if (rtd.readFault()) {
      printFaultsAndClear();
      return -273.15;//Ha habido un fallo
    }
    return rtd.readTemperatureC();
  }

  void guardarTemperatura(){
    ultimaTemp = leerTemperaturaC();
    arrayTemp[arrayTempPuntero] = ultimaTemp;
    arrayTempPuntero++;
    if(arrayTempPuntero >= tamanoArrayTemp){
      arrayTempPuntero = 0;
    }
  }
///////////////

///////////////Cosas de la radio
  LoRa_E220 e220ttl(&Serial2, 15, 21, 19);

  //Igual esto está mejor?
  //LoRa_E220 e220ttl(&Serial2, 22, 4, 18, 21, 19, UART_BPS_RATE_9600); //  esp32 RX <-- e220 TX, esp32 TX --> e220 RX AUX M0 M1

  //Para mandar mensajes habrá que usar sendCharArray
    //ResponseStatus rs = sendCharArray(payload, sizeof(payload));
///////////////

///////////////Máquina de estados
  enum estado{
    esperandoPruebaNueva,
    esperandoAlMotor,
    motorActivo,
    mandandoDatos
  };

  //habrá que ver qué se hace en cada estado 
///////////////


//SCK = GPIO 12
//MISO = GPIO 13
//MOSI = GPIO 11
//CS = GPIO 10

void setup() {
  
  Serial.begin(115200);
  delay(1);
  Serial.println("Serial encendido");

  SPI.begin(12,13,11,10);

  Serial.println("SPI encendido");

  setupADC();
  setupRTD();
  //e220ttl.begin();

  contadorPruebas2 = millis();
}

void loop() {

  if (ADCdataReady) {
    ADCdataReady = false;
    guardarFuerza();

    #if sacarMedidasPorSerial
      contadorPruebas++;
      contadorPruebas2++;
      if(contadorPruebas == 10){
        Serial.print("Variable_1:");
        //Serial.println(filtroFuerzas(arrayFuerzas),4);
        //Serial.println(ultimaFuerza,8);
        Serial.print("Variable_2:");
        Serial.println(0);
        //Serial.println((float)contadorPruebas2/(float)millis(),8);
        contadorPruebas = 0;
      }
    #endif

    divisorRateTemp++;

    // if(divisorRateTemp >= divisionRateTemperatura){
    //   divisorRateTemp = 0;
    //   guardarTemperatura();
    //   #if sacarMedidasPorSerial
    //     Serial.print("Variable_2: ");
    //     Serial.println(ultimaTemp);
    //   #endif
    // }
  }

  //delayMicroseconds(100);
  //delayMicroseconds(1);
}


float filtroFuerzas(float arrayFuerzas[]){
  float fuerza_media = 0;
  
  for (int i=0; i<1000; i++){
    fuerza_media += arrayFuerzas[i];
  }
  fuerza_media = fuerza_media/1000;
  if (fuerza_media == 0.1) return 0.0;
  else return fuerza_media;
}