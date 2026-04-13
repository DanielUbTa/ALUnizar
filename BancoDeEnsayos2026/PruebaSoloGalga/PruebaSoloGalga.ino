#include <Arduino.h>
#include <SPI.h>
#include <Protocentral_ADS1220.h>

// ── Pines SPI ──────────────────────────────────────────
#define SCK      12
#define MISO     13
#define MOSI     11
#define ADC_CS   10
#define ADC_DRDY  1

// ── Parámetros ADC / galga ──────────────────────────────
#define PGA        1
#define VREF       2.048f
#define FULL_SCALE ((1L << 23) - 1)

// Ajusta este factor a tu galga (300 kg fondo de escala en el original)
const float ESCALA_KG = 300.0f / (float)FULL_SCALE;
const float OFFSET_KG = 0.0f;

// ── Objeto ADC e ISR ────────────────────────────────────
Protocentral_ADS1220 adc;
volatile bool datosListos = false;

void IRAM_ATTR drdyISR() {
  datosListos = true;
}

// ── Setup ───────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  while (!Serial) {}
  Serial.println("=== Test Galga ADS1220 ===");

  SPI.begin(SCK, MISO, MOSI);

  pinMode(ADC_DRDY, INPUT);
  adc.begin(ADC_CS, ADC_DRDY);

  adc.set_pga_gain(PGA_GAIN_1);
  adc.set_data_rate(DR_1000SPS);
  adc.set_conv_mode_continuous();
  adc.Start_Conv();

  attachInterrupt(digitalPinToInterrupt(ADC_DRDY), drdyISR, FALLING);

  Serial.println("ADC iniciado. Leyendo...");
}

// ── Loop ────────────────────────────────────────────────
void loop() {
  if (datosListos) {
    datosListos = false;

    adc.Read_Data();
    int32_t raw    = adc.DataToInt();
    float   fuerza = raw * ESCALA_KG;//+ OFFSET_KG

    // Formato Serial Plotter: etiqueta:valor
    //Serial.print("Raw:");
    //Serial.print(raw);
    Serial.print("Fuerza_kg:");
    Serial.println(fuerza, 4);
    delay(10);
  }
}