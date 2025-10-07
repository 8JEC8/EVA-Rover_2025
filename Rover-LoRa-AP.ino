#include <SPI.h>
#include <LoRa.h>

#define LORA_FREQ 433E6

#define LORA_SS   5    // CS
#define LORA_DIO0 25   // DIO0

void setup() {
  Serial.begin(115200);
  while (!Serial);

  SPI.begin();

  LoRa.setPins(LORA_SS, -1, LORA_DIO0);

  if (!LoRa.begin(LORA_FREQ)) {
    Serial.println("AP_ERR_LoRa_FAILED");
    while (true);
  }

  // LoRa Settings:
  LoRa.setTxPower(20);
  LoRa.setSpreadingFactor(8); // Spreading Factor
  LoRa.setSignalBandwidth(125E3); // BW
  LoRa.setCodingRate4(5); // Coding Rate
  LoRa.setSyncWord(0x88); // Sync word
  LoRa.setPreambleLength(8); // Preamble: 8 symbols
  LoRa.enableCrc(); // CRC

  Serial.println("\n");
  Serial.println("Voyager21: LoRa Communication Ready");
  Serial.println("'.COMLIST' para Lista de Comandos Disponibles");
}

String msgToSend = "";    // Fila de mensaje de monitor serial
bool msgQueued = false;
unsigned long lastSendAttempt = 0;
const unsigned long retryInterval = 200; // ms entre intervalos de envío

void loop() {
  unsigned long now = millis();

  // RECV LoRa
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    String received = "";
    while (LoRa.available()) received += (char)LoRa.read();
    received.trim();
    Serial.println(String("  RECV_ROVER_") + String(LoRa.packetRssi()) + "," + received);

    // Revisar ACK: Check si mensaje sí se envió
    if (msgQueued && received.startsWith("ACK_")) {
      String ackFor = received.substring(4);
      if (ackFor == msgToSend) {
        msgQueued = false;
        msgToSend = "";
      }
    }
  }

  // Reintentar usando mensaje en Fila
  if (msgQueued && (now - lastSendAttempt >= retryInterval) && LoRa.beginPacket()) {
    LoRa.print(msgToSend);
    LoRa.endPacket();
    lastSendAttempt = now;
    Serial.println("SENT_EST_" + msgToSend);
  }

  // Comandos Fila
  while (Serial.available()) {
    char c = Serial.read();
    msgToSend += c;

    if (c == '\n') {
      msgToSend.trim();

      if (msgToSend.equalsIgnoreCase(".COMLIST")) {
        Serial.println("  Lista de Comandos:");
        Serial.println("    '.W'          : Movimiento Hacia Adelante");
        Serial.println("    '.S'          : Movimiento Hacia Atrás");
        Serial.println("    '.A'          : Movimiento CCW");
        Serial.println("    '.D'          : Movimiento CW");
        Serial.println("    '.CALCULATE'  : Calcula la ruta");
        Serial.println("    '.AUTO'       : Realiza la ruta de manera autonoma");
        Serial.println("    '.REVERSE'    : Realiza la ruta hacia de manera inversa");
        Serial.println("    '.SHOW'       : Muestra la ruta calculada");
        Serial.println("    '.INSTRUCTIONS': Muestra las instrucciones para realizar la ruta");
        Serial.println("    '.CHANGE'     : Cambiar la meta actual '.CHANGEY,X'");
        
        // Limpiar buffer
        msgToSend = "";
      } 
      else if (msgToSend.length() > 0) {
        msgQueued = true; // Meter en Fila
      }
    }
  }
}
