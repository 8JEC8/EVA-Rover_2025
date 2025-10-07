#include <SPI.h>
#include <LoRa.h>

#define LORA_FREQ 433E6
#define LORA_SS   5
#define LORA_DIO0 25

String imgBuffer = "";
int expectedChunks = 0;
int receivedChunks = 0;
bool receivingImage = false;

void setup() {
  Serial.begin(115200);
  while (!Serial);

  SPI.begin();
  LoRa.setPins(LORA_SS, -1, LORA_DIO0);

  if (!LoRa.begin(LORA_FREQ)) {
    Serial.println("AP_ERR_LoRa_FAILED");
    while (true);
  }

  // LoRa Settings
  LoRa.setTxPower(20);
  LoRa.setSpreadingFactor(10);
  LoRa.setSignalBandwidth(125E3);
  LoRa.setCodingRate4(5);
  LoRa.setSyncWord(0x88);
  LoRa.setPreambleLength(8);
  LoRa.enableCrc();

  Serial.println("\nVoyager21: LoRa Communication Ready");
  Serial.println("'.COMLIST' para Lista de Comandos Disponibles");
}

void loop() {
  // --- RECV: LoRa ---
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    String received = "";
    while (LoRa.available()) {
      received += (char)LoRa.read();
    }
    received.trim();

    // Handle image ready signal
    if (received.startsWith("IMG_READY")) {
      expectedChunks = received.substring(received.indexOf(',') + 1).toInt();
      receivedChunks = 0;
      imgBuffer = "";
      receivingImage = true;
      Serial.printf("EXPECTING_%d_CHUNKS\n", expectedChunks);

      // Start requesting chunks
      requestChunk(1);
    }

    // Handle simplified C_<chunk> packets with length and Base64 validation
    else if (received.startsWith("C_")) {
      if (receivingImage) {
        String chunkData = received.substring(2); // everything after "C_"

        // Determine expected length for this chunk
        int expectedLength = 128;
        if (receivedChunks == expectedChunks - 1) { // last chunk may be shorter
          expectedLength = -1; // allow any length <= 128 (or compute exact last chunk size if known)
        }

        // Check length
        bool lengthOK = (expectedLength == -1 && chunkData.length() <= 128) || (chunkData.length() == expectedLength);

        // Check Base64 characters (A-Z, a-z, 0-9, +, /, =)
        bool b64OK = true;
        for (int i = 0; i < chunkData.length(); i++) {
          char c = chunkData[i];
          if (!((c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') ||
                (c >= '0' && c <= '9') || c == '+' || c == '/' || c == '=')) {
            b64OK = false;
            break;
          }
        }

        if (lengthOK && b64OK) {
          // Chunk is valid
          imgBuffer += chunkData;
          receivedChunks++;
          Serial.printf("  RECV_CHUNK_%d/%d\n", receivedChunks, expectedChunks);

          if (receivedChunks < expectedChunks) {
            requestChunk(receivedChunks + 1);
          } else {
            // Image complete
            receivingImage = false;
            Serial.printf("IMG_RECEPTION_COMPLETE_%d\n", receivedChunks);
            Serial.println("B64_IMAGE_START");
            Serial.println(imgBuffer);
            Serial.println("B64_IMAGE_END");

            imgBuffer = "";
          }
        } else {
          // Chunk invalid → request again
          Serial.printf("CHUNK_%d_INVALID (len=%d, b64=%s) → REQUEST AGAIN\n",
                        receivedChunks + 1, chunkData.length(), b64OK ? "OK" : "FAIL");
          requestChunk(receivedChunks + 1);
        }
      }
    }

    else {
      // Normal message
      String received_message = "  RECV_ROVER_" + received + "_" + LoRa.packetRssi() + "_dBm";
      Serial.println(received_message);
    }
  }

  // --- SENT: Serial → LoRa ---
  if (Serial.available()) {
    String msg = Serial.readStringUntil('\n');
    msg.trim();

    if (msg.equalsIgnoreCase(".COMLIST")) {
      Serial.println(msg);
      Serial.println("  Lista de Comandos:");
      Serial.println("    '.RSSI'   : Intensidad de Señal Recibida");
      Serial.println("    '.AMBTEMP': Lectura de Temperatura y Humedad (Ambiente)");
      Serial.println("    '.INTTEMP': Lectura de Temperatura y Humedad (Interna)");
      Serial.println("    '.GYRO'   : Lectura de Acelerómetro y Giroscopio");
      Serial.println("    '.POWER#' : V,I,Pow ; # = 1 (ESP), 2 (Motor 1), 3 (Motor 3)");
      Serial.println("    '.DIST#'  : Distancia de Sensor de Laser # = 1, 2, o 3");
      Serial.println("    '.SET#'   : Selección de Pasos (200 = 1 revolución)");
      Serial.println("    '.W'      : Movimiento Hacia Adelante");
      Serial.println("    '.S'      : Movimiento Hacia Atrás");
      Serial.println("    '.A'      : Movimiento CCW");
      Serial.println("    '.D'      : Movimiento CW");
      Serial.println("    '.CALCULATE'    : Calcula la ruta");
      Serial.println("    '.AUTO'         : Realiza la ruta de manera autonoma");
      Serial.println("    '.REVERSE'      : Realiza la ruta hacia de manera inversa");
      Serial.println("    '.SHOW'         : Muestra la ruta caulculada");
      Serial.println("    '.INSTRUCTIONS' : Muestra las instrucciones para realizar la ruta");
      Serial.println("    '.CHANGE'       : Cambiar la meta actual '.CHANGEY,X'");
    } else {
      LoRa.beginPacket();
      LoRa.print(msg);
      LoRa.endPacket();

      Serial.print("SENT_EST_");
      Serial.println(msg);
    }
  }
}

void requestChunk(int seq) {
  LoRa.beginPacket();
  LoRa.print("REQ_" + String(seq));
  LoRa.endPacket();
  Serial.printf("REQUESTING_CHUNK_%d\n", seq);
}
