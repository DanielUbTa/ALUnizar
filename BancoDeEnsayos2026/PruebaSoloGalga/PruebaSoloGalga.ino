#include <Arduino.h>
#include <SPI.h>
#include <Protocentral_ADS1220.h>

// ── Pines SPI (Los definitivos de tu ESP32) ────────────
#define SCK       48
#define MISO      11
#define MOSI      45
#define ADC_CS    10
#define ADC_DRDY  4

// ── Parámetros ADC / galga ──────────────────────────────
#define PGA                 128
#define PGA_GAIN_DEFINE     PGA_GAIN_128
#define FULL_SCALE          (((long int)1<<23)-1)

// --- CÁLCULO CORRECTO DE LA ESCALA (Medición Ratiométrica) ---
// 1. Capacidad máxima de tu galga
const float CAPACIDAD_GALGA = 300.0f; 

// 2. Sensibilidad de la galga en V/V (2 mV/V = 0.002 V/V). 
// ¡Cámbialo a 0.001 o 0.003 si tu galga es de 1 o 3 mV/V!
const float SENSIBILIDAD = 0.001f; 

// 3. Al ser ratiométrico, los 5V se cancelan en la fórmula. El valor RAW máximo a 300kg es:
const float RAW_MAXIMO = FULL_SCALE * PGA * SENSIBILIDAD ; 


const float ESCALA_KG = 0.5* CAPACIDAD_GALGA / RAW_MAXIMO; // El 0.5 metido a mano y sacado de forma experimental
const float OFFSET_KG = 187.5f; // Aquí pondrás lo que pese tu estructura en vacío
// -------------------------------------------------------------

// ── Objeto ADC e ISR ────────────────────────────────────
Protocentral_ADS1220 adc;
volatile bool datosListos = false;

void IRAM_ATTR drdyISR() {
  datosListos = true;
}

// ── Setup ───────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(2000); 
  Serial.println("=== Test Galga ADS1220 (Modo Ratiometrico Datasheet) ===");

  pinMode(ADC_CS, OUTPUT);
  digitalWrite(ADC_CS, HIGH);

  SPI.begin(SCK, MISO, MOSI);
  delay(100);

  pinMode(ADC_DRDY, INPUT);
  adc.begin(ADC_CS, ADC_DRDY);

  SPI.begin(SCK, MISO, MOSI);

  // 1. Ganancia al máximo (x128)
  adc.set_pga_gain(PGA_GAIN_DEFINE); 
  
  // 2. ¡NUEVO! Leer la señal en AIN1 y AIN2 (Como en la Fig 82 del Datasheet)
  adc.select_mux_channels(MUX_AIN1_AIN2);
  
  // 3. ¡NUEVO! Usar AIN0 (5V) y AIN3 (GND) como referencia de voltaje
  // (Si VREF_EXT_REF1 te da error al compilar, sustitúyelo por un 2)
  adc.set_VREF(2); 

  adc.set_data_rate(DR_1000SPS);
  adc.set_conv_mode_continuous();
  adc.Start_Conv();

  attachInterrupt(digitalPinToInterrupt(ADC_DRDY), drdyISR, FALLING);

  // if (digitalRead(ADC_DRDY) == LOW) {
  //   adc.Read_Data(); 
  // }

  Serial.println("ADC iniciado. Leyendo...");
}

float filtroFuerza = 0.0;

// ── Loop ────────────────────────────────────────────────
void loop() {
  if (datosListos || digitalRead(ADC_DRDY) == LOW) {
    datosListos = false;

    adc.Read_Data();
    Serial.println("Datos leídos");
    int32_t raw    = adc.DataToInt();
    float   fuerza = (raw * ESCALA_KG) - OFFSET_KG;
    filtroFuerza += 0.001*(fuerza-filtroFuerza);

    // Imprimimos el RAW para depurar y los Kilos reales
    Serial.print("Raw:");
    Serial.print(raw);
    Serial.print(" %ADC:");
    Serial.print(100.0*float(raw)/float(FULL_SCALE));
    Serial.print(" | Fuerza_kg:");
    Serial.println(fuerza, 4);
  }
}