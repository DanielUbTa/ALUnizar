#include <Arduino.h>
#include <SPI.h>
#include <Protocentral_ADS1220.h>
#include <7Semi_MAX31865.h>

int contadorPruebas = 0;
int contadorPruebas2 = 0;

#define sacarMedidasPorSerial true

// ================= PINES SPI =================
#define SCK 12
#define MISO 13
#define MOSI 11
#define ADC_CS 10    // CS de la Galga
#define RTD_CS_PIN 35 // CS de la PT100
#define ADC_DRDY 1   // Data Ready del ADC
// =============================================

/////////////// Globales para guardado de datos
#define tamanoArrayFuerzas 1000 
float arrayFuerzas[tamanoArrayFuerzas];
uint32_t arrayFuerzasPuntero = 0;

#define divisionRateTemperatura 100
int divisorRateTemp = 0;

#define tamanoArrayTemp 256
float arrayTemp[tamanoArrayTemp];
uint32_t arrayTempPuntero = 0;
///////////////

/////////////// Cosas del ADC (Galga directa)
#define PGA          128               // ¡Cambiado a 128 porque ya no hay amplificador externo!
#define VREF         2.048             // Referencia interna de 2.048V
#define VFSR         VREF/PGA
#define FULL_SCALE   (((long int)1<<23)-1)

const float mult = 300.0 / (FULL_SCALE); // Multiplicar por 9.81 para sacar los newtons
const float suma = 0.0;

float ultimaFuerza = 0.0;

Protocentral_ADS1220 ads1220;
volatile bool ADCdataReady = false;

// Interrupción del ADC
void IRAM_ATTR drdyISR() {
  ADCdataReady = true;
}

void setupADC() {
  pinMode(ADC_DRDY, INPUT);
  ads1220.begin(ADC_CS, ADC_DRDY);

  // Ganancia a 128 para leer los milivoltios de la galga directamente
  ads1220.set_pga_gain(PGA_GAIN_128); 
  ads1220.set_data_rate(DR_1000SPS);
  ads1220.set_conv_mode_continuous();
  ads1220.Start_Conv();

  attachInterrupt(digitalPinToInterrupt(ADC_DRDY), drdyISR, FALLING);
}

void guardarFuerza() {
  ads1220.Read_Data();
  ultimaFuerza = ((float)ads1220.DataToInt()) * mult + suma; 
  arrayFuerzas[arrayFuerzasPuntero] = ultimaFuerza;
  arrayFuerzasPuntero++;

  if(arrayFuerzasPuntero >= tamanoArrayFuerzas){
    arrayFuerzasPuntero = 0;
  }
}
///////////////

/////////////// Cosas del termopar (PT100)
#define RREF_OHM     430.0f
#define R0_OHM       100.0f  

float ultimaTemp = 0.0;

MAX31865_7Semi rtd(RTD_CS_PIN, SPI);

void printFaultsAndClear() {
  Serial.println(F("MAX31865: FAULT detected!"));
  MAX31865_7Semi::FaultStatus f = rtd.readFaultStatus();
  if (f.rtdHigh)       Serial.println(F("  - RTD en circuito abierto")); 
  if (f.rtdLow)        Serial.println(F("  - RTD en cortocircuito")); 
  if (f.refInHigh)     Serial.println(F("  - REFIN- > 0.85*Vbias"));
  if (f.refInLow)      Serial.println(F("  - REFIN- < 0.85*Vbias"));
  if (f.rtdInLow)      Serial.println(F("  - RTDIN- < 0.85*Vbias"));
  if (f.overUnderVolt) Serial.println(F("  - Over/Under Voltage"));
  rtd.clearFaults();
}

void setupRTD() {
  rtd.begin(WIRES_3,      
            FILTER_50HZ,  
            true,         
            true,         
            1000000);     

  rtd.setReferenceResistor(RREF_OHM);
  rtd.setR0(R0_OHM);

  rtd.setLowThreshold(20.0f);
  rtd.setHighThreshold(300.0f);
  rtd.clearFaults();
}

float leerTemperaturaC() {
  if (rtd.readFault()) {
    printFaultsAndClear();
    return -273.15; // Ha habido un fallo
  }
  return rtd.readTemperatureC();
}

void guardarTemperatura() {
  ultimaTemp = leerTemperaturaC();
  arrayTemp[arrayTempPuntero] = ultimaTemp;
  arrayTempPuntero++;
  if(arrayTempPuntero >= tamanoArrayTemp){
    arrayTempPuntero = 0;
  }
}
///////////////

/////////////// Funciones de Filtro
float filtroFuerzas(float arrayFuerzas[]){
  float fuerza_media = 0;
  for (int i=0; i<tamanoArrayFuerzas; i++){
    fuerza_media += arrayFuerzas[i];
  }
  return fuerza_media / tamanoArrayFuerzas;
}

float filtroTemperatura(float arrayTemp[]){
  float temp_media = 0;
  for (int i=0; i<tamanoArrayTemp; i++){
    temp_media += arrayTemp[i];
  }
  return temp_media / tamanoArrayTemp;
}
///////////////

void setup() {
  Serial.begin(115200);
  delay(2000); // Tiempo para que abra el monitor serie
  Serial.println("\n--- INICIANDO SENSORES ---");

  // 1. SILENCIAR A LOS CHIPS (Evita conflictos SPI al arrancar)
  pinMode(ADC_CS, OUTPUT);
  digitalWrite(ADC_CS, HIGH);
  pinMode(RTD_CS_PIN, OUTPUT);
  digitalWrite(RTD_CS_PIN, HIGH);

  // 2. INICIAR SPI
  SPI.begin(SCK, MISO, MOSI);
  delay(100);

  // 3. CONFIGURAR GALGA
  Serial.println("Iniciando ADC (Galga)...");
  setupADC();

  // 4. RE-FORZAR PINES SPI (Por si la librería del ADC los cambió)
  SPI.begin(SCK, MISO, MOSI);

  // 5. CONFIGURAR PT100
  Serial.println("Iniciando RTD (PT100)...");
  setupRTD();

  // 6. RE-FORZAR PINES SPI (Por si la librería del RTD los cambió)
  SPI.begin(SCK, MISO, MOSI);

  // 7. DESATASCAR ADC (Si se quedó con un dato pendiente)
  if (digitalRead(ADC_DRDY) == LOW) {
    ads1220.Read_Data(); 
  }

  Serial.println("¡Setup Completado! Leyendo datos...");
  contadorPruebas2 = millis();
}

void loop() {
  // A PRUEBA DE BALAS: Interrupción O pin en LOW
  if (ADCdataReady || digitalRead(ADC_DRDY) == LOW) {
    ADCdataReady = false;
    
    guardarFuerza();

    #if sacarMedidasPorSerial
      contadorPruebas++;
      if(contadorPruebas == 10){
        Serial.print("Fuerza(Kg): ");
        Serial.println(filtroFuerzas(arrayFuerzas), 4);
        contadorPruebas = 0;
      }
    #endif 

    divisorRateTemp++;
   
    if(divisorRateTemp >= divisionRateTemperatura){
      divisorRateTemp = 0;
      guardarTemperatura();
      
      #if sacarMedidasPorSerial
        Serial.print("Temperatura: ");
        Serial.println(filtroTemperatura(arrayTemp));
      #endif
    }
  }

  delayMicroseconds(100);
}