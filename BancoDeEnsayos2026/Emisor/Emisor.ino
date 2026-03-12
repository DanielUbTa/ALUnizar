#include <Arduino.h>
#include <SPI.h>
#include <Protocentral_ADS1220.h>
#include <7Semi_MAX31865.h>
#include "LoRa_E220.h"

int contadorPruebas = 0;
#define mandarMedidasPorLoRa true

// Pines SPI comunes
#define SCK 12
#define MISO 13
#define MOSI 11
#define CS_Galga 10 // Es activo en bajo

/////////////// Globales para guardado de datos
#define tamanoArrayFuerzas 1000 
float arrayFuerzas[tamanoArrayFuerzas];
uint32_t arrayFuerzasPuntero = 0;

// Configuración de frecuencia de envío por LoRa
// LoRa es más lento que el cable. 
// 200 lecturas = aprox 5 Hz para Fuerza
// 1000 lecturas = aprox 1 Hz para Temperatura
#define lecturasParaFuerza 200       
#define lecturasParaTemp 1000        
int divisorRateTemp = 0;

#define tamanoArrayTemp 256
float arrayTemp[tamanoArrayTemp];
uint32_t arrayTempPuntero = 0;

/////////////// Cosas del ADC
#define PGA          1                 
#define VREF         2.048            
#define VFSR         VREF/PGA
#define FULL_SCALE   (((long int)1<<23)-1)

const float mult = 0.00001788139;
const float suma = 0.0;
float ultimaFuerza = 0.0;

#define ADC_CS   10
#define ADC_DRDY 1
Protocentral_ADS1220 ads1220;
volatile bool ADCdataReady = false;

void IRAM_ATTR drdyISR() {
  ADCdataReady = true;
}

void setupADC(){
  pinMode(ADC_DRDY, INPUT);
  ads1220.begin(ADC_CS, ADC_DRDY);
  ads1220.set_pga_gain(PGA_GAIN_1); 
  ads1220.set_data_rate(DR_1000SPS);
  ads1220.set_conv_mode_continuous();
  ads1220.Start_Conv();
  attachInterrupt(digitalPinToInterrupt(ADC_DRDY), drdyISR, FALLING);
}

void guardarFuerza(){
  ads1220.Read_Data();
  ultimaFuerza = ((float)ads1220.DataToInt()) * mult + suma; 
  arrayFuerzas[arrayFuerzasPuntero] = ultimaFuerza;
  arrayFuerzasPuntero++;
  if(arrayFuerzasPuntero >= tamanoArrayFuerzas){
    arrayFuerzasPuntero = 0;
  }
}

/////////////// Cosas del termopar
#define RTD_CS_PIN   35
#define RREF_OHM     430.0f
#define R0_OHM       100.0f  

float ultimaTemp = 0.0;
MAX31865_7Semi rtd(RTD_CS_PIN, SPI);

// Inicializamos objeto LoRa (TX2=17, RX2=16 normalmente se usan internamente en Serial2)
LoRa_E220 e220ttl(&Serial2, 36);

void printFaultsAndClear() {
  // Juntamos los errores en un solo string para no colapsar la red LoRa
  String errorMsg = "MAX31865: FAULT detectado! ";
  MAX31865_7Semi::FaultStatus f = rtd.readFaultStatus();
  if (f.rtdHigh)       errorMsg += "[RTD abierto] ";
  if (f.rtdLow)        errorMsg += "[RTD en corto] ";
  if (f.refInHigh)     errorMsg += "[REFIN- Alto] ";
  if (f.refInLow)      errorMsg += "[REFIN- Bajo] ";
  if (f.rtdInLow)      errorMsg += "[RTDIN- Bajo] ";
  if (f.overUnderVolt) errorMsg += "[Over/Under Volt] ";
  
  e220ttl.sendMessage(errorMsg);
  rtd.clearFaults();
}

void setupRTD() {
  rtd.begin(WIRES_3, FILTER_50HZ, true, true, 1000000);
  rtd.setReferenceResistor(RREF_OHM);
  rtd.setR0(R0_OHM);
  rtd.setLowThreshold(20.0f);
  rtd.setHighThreshold(40.0f);
  rtd.clearFaults();
}

float leerTemperaturaC() {
  if (rtd.readFault()) {
    printFaultsAndClear();
    return -273.15; 
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

/////////////// Declaración previa de funciones
float filtroFuerzas(float arr[]);
float filtroTemperatura(float arr[]);

void setup() {
  Serial.begin(115200); // Debug opcional
  delay(1);
  
  SPI.begin(SCK, MISO, MOSI);
  
  setupADC();
  setupRTD();
  
  // Encendemos el módulo LoRa
  e220ttl.begin();
}

void loop() {
  if (ADCdataReady) {
    ADCdataReady = false;
    guardarFuerza();

    #if mandarMedidasPorLoRa
      contadorPruebas++;
      if(contadorPruebas >= lecturasParaFuerza){
        // Construimos el string y lo enviamos
        String msgFuerza = "Fuerza(Kg):" + String(filtroFuerzas(arrayFuerzas), 4);
        e220ttl.sendMessage(msgFuerza);
        contadorPruebas = 0;
      }
    #endif 

    divisorRateTemp++;
    if(divisorRateTemp >= lecturasParaTemp){
      divisorRateTemp = 0;
      guardarTemperatura();
      
      #if mandarMedidasPorLoRa
        String msgTemp = "Temperatura:" + String(filtroTemperatura(arrayTemp), 2);
        e220ttl.sendMessage(msgTemp);
      #endif
    }
  }

  delayMicroseconds(100);
}

float filtroFuerzas(float arr[]){
  float media = 0;
  for (int i=0; i<tamanoArrayFuerzas; i++){
    media += arr[i];
  }
  return media/tamanoArrayFuerzas;
}

float filtroTemperatura(float arr[]){
  float media = 0;
  for (int i=0; i<tamanoArrayTemp; i++){
    media += arr[i];
  }
  return media/tamanoArrayTemp;
}