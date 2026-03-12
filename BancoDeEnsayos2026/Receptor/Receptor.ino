#include <Arduino.h>
#include "LoRa_E220.h"

// Inicializamos objeto LoRa para el receptor
// Revisa si usas los mismos pines (AUX 15, M0 21, M1 19) en tu placa RX
LoRa_E220 e220ttl(&Serial2, 32);

void setup() {
  // Iniciamos la comunicación con el PC por USB (Importante que sea a 115200)
  Serial.begin(115200);
  delay(10);
  
  // Iniciar LoRa
  e220ttl.begin();
}

void loop() {
  // Si hay datos entrantes desde el emisor LoRa...
  if (e220ttl.available() > 1) {
    
    // Leemos el mensaje recibido del aire
    ResponseContainer rc = e220ttl.receiveMessage();
    
    // Verificar si se leyó correctamente (code 1 = Éxito)
    if (rc.status.code == 1) {
      // Imprimimos tal cual el string al puerto serie hacia Python
      Serial.println(rc.data);
    } 
  }
}