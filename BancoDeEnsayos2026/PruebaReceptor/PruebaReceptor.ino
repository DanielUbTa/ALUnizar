#include <Arduino.h>
#include "LoRa_E220.h"

// Pines del ESP32-S3-Zero (Receptor)
#define RX_PIN 44 // Conectar al TX del LoRa
#define TX_PIN 43 // Conectar al RX del LoRa
#define AUX_PIN 6
#define M1_PIN 7
#define M0_PIN 8

// Inicializamos LoRa pasándole los pines M0 y M1 para que la librería los controle
LoRa_E220 e220ttl(&Serial2, AUX_PIN, M0_PIN, M1_PIN);

void setup() {
  Serial.begin(115200);
  delay(3000); // Tiempo para que conecte el USB
  
  Serial.println("\n--- TEST BASICO RECEPTOR ---");

  Serial2.begin(9600, SERIAL_8N1, RX_PIN, TX_PIN);
  
  // Iniciar LoRa
  e220ttl.begin();
  
  // ¡MUY IMPORTANTE! Forzamos al módulo a entrar en Modo Normal (Transmisión/Recepción)
  // Esto pone M0 y M1 a LOW internamente.
  e220ttl.setMode(MODE_0_NORMAL);

  Serial.println("Receptor iniciado y escuchando el aire...");
}

void loop() {
  // Si el pin AUX nos avisa de que hay datos o el buffer tiene algo...
  if (e220ttl.available() > 0) {
    
    Serial.println("¡Señal detectada en el aire!");
    
    // Leemos el mensaje
    ResponseContainer rc = e220ttl.receiveMessage();
    
    // Comprobamos si llegó bien
    if (rc.status.code == 1) {
      Serial.print("Mensaje recibido: ");
      Serial.println(rc.data);
      Serial.print("RSSI (Fuerza de señal): ");
      Serial.println(rc.rssi);
    } else {
      Serial.print("Error al decodificar: ");
      Serial.println(rc.status.getResponseDescription());
    }
  }
}