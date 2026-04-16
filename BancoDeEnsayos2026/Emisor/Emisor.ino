#include <Arduino.h>
#include <SPI.h>
#include <Protocentral_ADS1220.h>
#include <7Semi_MAX31865.h>
#include "LoRa_E220.h"

#define ENVIAR_POR_LORA 1

// ===================== PINES DEFINITIVOS =====================
// Pines SPI
#define SCK   48
#define MISO  11 
#define MOSI  45

// Pines LoRa
#define LORA_RX_PIN 42 // Conectar al TX del LoRa
#define LORA_TX_PIN 2  // Conectar al RX del LoRa
#define LORA_AUX_PIN 37
#define M1_PIN 21
#define M0_PIN 47

// ADS1220 (Fuerza)
#define ADC_CS    10
#define ADC_DRDY  4

// RTD MAX31865 (Temperatura)
#define RTD_CS_PIN 35 
#define RREF_OHM   430.0f
#define R0_OHM     100.0f
// ==============================================================

// Configuración de envío
#define LECTURAS_PARA_ENVIO_FUERZA 200   
#define LECTURAS_PARA_TEMP         1000  

// Variables ADC fuerza 
#define PGA                 128
#define PGA_GAIN_DEFINE     PGA_GAIN_128
#define VREF                2.048
#define VFSR                VREF / PGA
#define FULL_SCALE          (((long int)1 << 23) - 1)
const float SENSIBILIDAD = 0.001f;

const float mult = 150.0f/(FULL_SCALE * PGA * SENSIBILIDAD);
const float suma = 187.5f;

Protocentral_ADS1220 ads1220;
volatile bool ADCdataReady = false;

float sumaFuerzas = 0.0f;
uint32_t contadorFuerzas = 0;
float ultimaFuerza = 0.0f;

// Variables RTD
MAX31865_7Semi rtd(RTD_CS_PIN, SPI);
float ultimaTemp = NAN;

// LoRa
LoRa_E220 e220ttl(&Serial2, LORA_AUX_PIN, M0_PIN, M1_PIN);

// Control
uint32_t divisorRateTemp = 0;
uint32_t numeroPaquete = 0;

// -------------------------------------------------------
void IRAM_ATTR drdyISR() {
  ADCdataReady = true;
}

// -------------------------------------------------------
void setupADC() {
  pinMode(ADC_DRDY, INPUT);
  ads1220.begin(ADC_CS, ADC_DRDY);
  ads1220.set_pga_gain(PGA_GAIN_128);
  ads1220.select_mux_channels(MUX_AIN1_AIN2);
  ads1220.set_VREF(2);
  ads1220.set_data_rate(DR_1000SPS);
  ads1220.set_conv_mode_continuous();
  ads1220.Start_Conv();

  attachInterrupt(digitalPinToInterrupt(ADC_DRDY), drdyISR, FALLING);
}

// -------------------------------------------------------
void setupRTD() {
  rtd.begin(WIRES_3, FILTER_50HZ, true, true, 1000000);
  rtd.setReferenceResistor(RREF_OHM);
  rtd.setR0(R0_OHM);
  rtd.clearFaults();
}

// -------------------------------------------------------
void printFaultsAndClear() {
  String errorMsg = "ERROR_RTD:";
  MAX31865_7Semi::FaultStatus f = rtd.readFaultStatus();

  if (f.rtdHigh)       errorMsg += "[RTD_ABIERTO]";
  if (f.rtdLow)        errorMsg += "[RTD_CORTO]";
  if (f.refInHigh)     errorMsg += "[REFIN_ALTO]";
  if (f.refInLow)      errorMsg += "[REFIN_BAJO]";
  if (f.rtdInLow)      errorMsg += "[RTDIN_BAJO]";
  if (f.overUnderVolt) errorMsg += "[OV_UV]";

#if ENVIAR_POR_LORA
  ResponseStatus rs = e220ttl.sendMessage(errorMsg);
  Serial.print("Envio error RTD: ");
  Serial.println(rs.getResponseDescription());
#endif

  rtd.clearFaults();
}

// -------------------------------------------------------
float leerTemperaturaC() {
  if (rtd.readFault()) {
    printFaultsAndClear();
    return NAN;
  }
  return rtd.readTemperatureC();
}

// -------------------------------------------------------
void leerYAcumularFuerza() {
  ads1220.Read_Data();
  ultimaFuerza = ((float)ads1220.DataToInt()) * mult - suma;
  sumaFuerzas += ultimaFuerza;
  contadorFuerzas++;
}

// -------------------------------------------------------
void actualizarTemperatura() {
  ultimaTemp = leerTemperaturaC();
}

// -------------------------------------------------------
void enviarPaquete(float fuerzaMedia, float temperatura) {
  char msg[96];

  if (isnan(temperatura)) {
    snprintf(msg, sizeof(msg), "%lu,%.4f,NAN\n", (unsigned long)numeroPaquete, fuerzaMedia);
  } else {
    snprintf(msg, sizeof(msg), "%lu,%.4f,%.2f\n", (unsigned long)numeroPaquete, fuerzaMedia, temperatura);
  }

#if ENVIAR_POR_LORA
  ResponseStatus rs = e220ttl.sendMessage(msg);
  Serial.print("Estado de Respuesta: ");
  Serial.println(rs.code);
  Serial.print("TX -> ");
  Serial.println(msg);

  if (rs.code != 1) {
    Serial.print("Error envio LoRa: ");
    Serial.println(rs.getResponseDescription());
  }
#endif
}


void setup() {
  Serial.begin(115200);
  
  // IMPORTANTE: Dar tiempo al USB nativo para que no se desconecte
  delay(3000); 
  Serial.println("\n--- INICIANDO SISTEMA ---");

  // Iniciar Serial2 con los pines del LoRa
  Serial2.begin(9600, SERIAL_8N1, LORA_RX_PIN, LORA_TX_PIN);
  delay(100);

  // Iniciar el bus SPI con los pines correctos
  
  pinMode(ADC_CS, OUTPUT);
  digitalWrite(ADC_CS, HIGH);
  

  SPI.begin(SCK, MISO, MOSI);
  setupADC();
  SPI.begin(SCK, MISO, MOSI);
  setupRTD();
  SPI.begin(SCK, MISO, MOSI);
  Serial.println("Iniciando LoRa...");
  bool testLoRa = e220ttl.begin();
  ResponseStructContainer c = e220ttl.getConfiguration();
  if (c.status.code == 1) {
    Configuration config = *(Configuration*) c.data;
    if (config.CHAN != 18) {
      Serial.println("Cambiando el canal al 18 para coincidir con el Emisor...");
      config.CHAN = 18;
      e220ttl.setConfiguration(config, WRITE_CFG_PWR_DWN_SAVE);
      Serial.println("¡Canal actualizado!");
    } else {
      Serial.println("El canal ya es el 18. ¡Perfecto!");
    }
  }
  e220ttl.setMode(MODE_0_NORMAL);
  delay(5000);
  if(testLoRa) Serial.println("LoRa iniciado correctamente");

  Serial.println("Transmisor listo y funcionando");
}

void loop() {
  if (ADCdataReady) {
    ADCdataReady = false;

    leerYAcumularFuerza();

    divisorRateTemp++;
    if (divisorRateTemp >= LECTURAS_PARA_TEMP) {
      divisorRateTemp = 0;
      actualizarTemperatura();
    }
    // Serial.print("Contador Fuerzas: ");
    // Serial.println(contadorFuerzas);
    if (contadorFuerzas >= LECTURAS_PARA_ENVIO_FUERZA) {
      float fuerzaMedia = sumaFuerzas / (float)contadorFuerzas;
      Serial.println("Enviando mensaje");
      enviarPaquete(fuerzaMedia, ultimaTemp);
      sumaFuerzas = 0.0f;
      contadorFuerzas = 0;
      numeroPaquete++;
    }
  }
}