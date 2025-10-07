#include <SPI.h>
#include <LoRa.h>

#define LORA_FREQ 433E6
#define LORA_SS   5    // CS
#define LORA_DIO0 25   // DIO0

String imgBuffer = "";
int expectedChunks = 0;
int receivedChunks = 0;
bool receivingImage = false;

String msgToSend = "";    // Fila de mensaje de monitor serial
bool msgQueued = false;
unsigned long lastSendAttempt = 0;
const unsigned long retryInterval = 500; // ms entre intervalos de envío

const unsigned long ackTimeout = 5000; // 5 segundo de espera en reintento
unsigned long msgStartTime = 0;        // Cuando el mensaje fue enviado

unsigned long lastChunkRequestTime = 0;
const unsigned long chunkTimeout = 5000; // 3 segundos para reenviar solicitud de chunk

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
  LoRa.setSpreadingFactor(9); // Spreading Factor
  LoRa.setSignalBandwidth(125E3); // BW
  LoRa.setCodingRate4(5); // Coding Rate
  LoRa.setSyncWord(0x88); // Sync word
  LoRa.setPreambleLength(8); // Preamble: 8 symbols
  LoRa.enableCrc(); // CRC

  Serial.println("\nVoyager21: Comunicación LoRa Habillitada");
  Serial.println("'.COMLIST' para Lista de Comandos Disponibles");
}

void loop() {
  unsigned long now = millis();

  // --- RECEIVE: LoRa ---
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    String received = "";
    while (LoRa.available()) received += (char)LoRa.read();
    received.trim();
    Serial.println(String("  RECV_ROVER_") + String(LoRa.packetRssi()) + "," + received);

    if (msgQueued && received.startsWith("ACK_")) {
      String ackFor = received.substring(4);
      if (ackFor == msgToSend) {
        msgQueued = false;
        msgToSend = "";

        if (receivingImage && ackFor.startsWith("REQ_")) {
            lastChunkRequestTime = now;
        }
      }
    }

    // Handle image ready signal
    if (received.startsWith("IMG_READY")) {
      expectedChunks = received.substring(received.indexOf(',') + 1).toInt();
      receivedChunks = 0;
      imgBuffer = "";
      receivingImage = true;
      Serial.printf("EXPECTING_%d_CHUNKS\n", expectedChunks);

      // Start requesting first chunk
      msgToSend = "REQ_1";
      msgQueued = true;
      lastSendAttempt = now;
      msgStartTime = now;
      lastChunkRequestTime = now;
    }

    // Handle chunk data
    else if (received.startsWith("C_")) {
      if (receivingImage) {
        String chunkData = received.substring(2);

        int expectedLength = 128;
        if (receivedChunks == expectedChunks - 1) expectedLength = -1;

        bool lengthOK = (expectedLength == -1 && chunkData.length() <= 128) ||
                        (chunkData.length() == expectedLength);

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
          imgBuffer += chunkData;
          receivedChunks++;
          Serial.printf("  RECV_CHUNK_%d/%d\n", receivedChunks, expectedChunks);

          // Request next chunk
          if (receivedChunks < expectedChunks) {
            msgToSend = "REQ_" + String(receivedChunks + 1);
            msgQueued = true;
            lastSendAttempt = now;
            msgStartTime = now;
          } else {
            receivingImage = false;
            Serial.printf("IMG_RECEPTION_COMPLETE_%d\n", receivedChunks);
            Serial.println("B64_IMAGE_START");
            Serial.println(imgBuffer);
            Serial.println("B64_IMAGE_END");
            imgBuffer = "";
          }
        } else {
          Serial.printf("CHUNK_%d_INVALID(len=%d,b64=%s) → REQUEST AGAIN\n",
                        receivedChunks + 1, chunkData.length(), b64OK ? "OK" : "FAIL");
          msgToSend = "REQ_" + String(receivedChunks + 1);
          msgQueued = true;
          lastSendAttempt = now;
          msgStartTime = now;
          lastChunkRequestTime = now;
        }
      }
    }
  }

  // --- RETRY queued message with timeout ---
  if (msgQueued) {
    // Timeout: stop retries
    if (now - msgStartTime >= ackTimeout) {
      Serial.println("  MSG_TIMEOUT");
      msgQueued = false;
      msgToSend = "";
    }
    // Retry if interval passed
    else if (now - lastSendAttempt >= retryInterval && LoRa.beginPacket()) {
      LoRa.print(msgToSend);
      LoRa.endPacket();
      lastSendAttempt = now;
      Serial.println("SENT_EST_" + msgToSend);
    }
  }

  if (receivingImage && !msgQueued && (now - lastChunkRequestTime >= chunkTimeout)) {
    Serial.printf("CHUNK_%d_TIMEOUT_REREQUESTING\n", receivedChunks + 1);
    msgToSend = "REQ_" + String(receivedChunks + 1);
    msgQueued = true;
    lastSendAttempt = now;
    msgStartTime = now;
    lastChunkRequestTime = now;
  }

  // --- Serial command queue ---
  while (Serial.available()) {
    char c = Serial.read();
    msgToSend += c;

    if (c == '\n') {
      msgToSend.trim();

      if (msgToSend.equalsIgnoreCase(".COMLIST")) {
        Serial.println("  Lista de Comandos:");
        Serial.println("    '.W'            : Movimiento Hacia Adelante");
        Serial.println("    '.S'            : Movimiento Hacia Atrás");
        Serial.println("    '.A'            : Movimiento CCW");
        Serial.println("    '.D'            : Movimiento CW");
        Serial.println("    '.IMAGE'        : Captura de Imagen y Recepción en Chunks");
        Serial.println("    '.CALCULATE'    : Calcula la ruta");
        Serial.println("    '.AUTO'         : Realiza la ruta de manera autonoma");
        Serial.println("    '.REVERSE'      : Realiza la ruta hacia de manera inversa");
        Serial.println("    '.SHOW'         : Muestra la ruta calculada");
        Serial.println("    '.INSTRUCTIONS' : Muestra las instrucciones para realizar la ruta");
        Serial.println("    '.CHANGE'       : Cambiar la meta actual '.CHANGEY,X'");
        msgToSend = "";
      } else if (msgToSend.length() > 0) {
        msgQueued = true;
        msgStartTime = now;
        lastSendAttempt = now;
      }
    }
  }
}

void requestChunk(int seq) {
  LoRa.beginPacket();
  LoRa.print("REQ_" + String(seq));
  LoRa.endPacket();
  Serial.printf("REQUESTING_CHUNK_%d\n", seq);
}
