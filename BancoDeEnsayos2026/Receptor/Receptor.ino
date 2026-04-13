#include <Arduino.h>
#include "LoRa_E220.h"

// Definición de pines para el ESP32-S3-Zero
#define RX_PIN 44 // Pin RX de la placa (conectar al TX del LoRa)
#define TX_PIN 43 // Pin TX de la placa (conectar al RX del LoRa)
#define AUX_PIN 6
#define M1_PIN 7
#define M0_PIN 8

// Inicializamos objeto LoRa pasándole Serial2, AUX, M0 y M1
// Al pasar M0 y M1, la librería gestiona los modos de energía automáticamente
LoRa_E220 e220ttl(&Serial2, AUX_PIN, M0_PIN, M1_PIN);

void setup() {
  // Iniciamos la comunicación con el PC por USB
  Serial.begin(115200);
  
  // El ESP32-S3 usa USB nativo, le damos un par de segundos para que 
  // el monitor serie del PC tenga tiempo de conectarse y no perder los primeros mensajes.
  delay(2000); 
  
  Serial.println("Configurando puerto serie para LoRa...");
  
  // Iniciamos Serial2 explícitamente con los pines del ESP32-S3-Zero
  // El E220 usa 8N1 por defecto a 9600 baudios
  Serial2.begin(9600, SERIAL_8N1, RX_PIN, TX_PIN);
  
  Serial.println("Iniciando modulo LoRa E220...");
  
  // Iniciar LoRa
  e220ttl.begin();

  Serial.println("Receptor listo y escuchando...");
}

void loop() {
  // Si hay datos entrantes desde el emisor LoRa...
  if (e220ttl.available() > 0) {
    
    // Leemos el mensaje recibido del aire
    ResponseContainer rc = e220ttl.receiveMessage();
    
    // Verificar si se leyó correctamente (code 1 = Éxito)
    if (rc.status.code == 1) {
      Serial.print("Mensaje recibido: ");
      Serial.println(rc.data);
    } else {
      Serial.print("Error RX: ");
      Serial.println(rc.status.getResponseDescription());
    }
  }
}