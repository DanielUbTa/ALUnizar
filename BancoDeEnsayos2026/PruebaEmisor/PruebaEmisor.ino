#include <Arduino.h>
#include "LoRa_E220.h"

// Pines LoRa
#define LORA_RX_PIN 42 // Conectar al TX del LoRa
#define LORA_TX_PIN 2  // Conectar al RX del LoRa
#define LORA_AUX_PIN 37 
#define M1_PIN 21
#define M0_PIN 47


// Inicializamos LoRa
LoRa_E220 e220ttl(&Serial2, LORA_AUX_PIN, M0_PIN, M1_PIN);

int contador = 0;

void setup() {
  Serial.begin(115200);
  delay(3000); // Tiempo para el USB
  
  Serial.println("\n--- TEST BASICO LORA E220 ---");

  // Iniciar Serial2
  Serial2.begin(9600, SERIAL_8N1, LORA_RX_PIN, LORA_TX_PIN);
  
  // Iniciar LoRa
  e220ttl.begin();
  e220ttl.setMode(MODE_0_NORMAL);
  
  Serial.println("LoRa Iniciado. Empezando a transmitir...");
}

void loop() {
  String mensaje = "Prueba de conexion: " + String(contador);
  
  // Enviar mensaje
  ResponseStatus rs = e220ttl.sendMessage(mensaje);
  
  // Imprimir en el monitor serie lo que está pasando
  Serial.print("Enviando: ");
  Serial.print(mensaje);
  Serial.print(" | Estado del modulo: ");
  Serial.println(rs.getResponseDescription()); // Debería decir "Success"
  
  contador++;
  delay(3000); // Esperar 3 segundos
}