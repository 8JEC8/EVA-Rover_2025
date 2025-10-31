#include <SPI.h>
#include <LoRa.h>
#include <DFRobotDFPlayerMini.h>
#include <WiFi.h>
#include <WebSocketsServer.h>
#include <WebServer.h> 
#define LORA_FREQ 433E6
#define LORA_SS   5    // CS
#define LORA_DIO0 25   // DIO0

HardwareSerial dfSerial(2);
DFRobotDFPlayerMini myDFPlayer;

const char* apSSID = "EVA_Dashboard";
const char* apPassword = "12345678";  // min 8 chars

WebServer server(80);
WebSocketsServer webSocket = WebSocketsServer(81);

// Only first client can control
uint8_t controllerClient = 255; // 255 = no controller yet

struct Track {
  int folder;
  int track;
};

Track trackQueue[10];
int queueSize = 0;

const int punFolder = 3;                  // Example folder for puns
const int punTracks[] = {
  1, 2, 3, 4, 5, 6, 7, 8, 9, 10,
  11,12,13,14,15
};
const int numPunTracks = sizeof(punTracks)/sizeof(punTracks[0]);

// Timer
unsigned long lastPunTime = 0;
const unsigned long punInterval = 5UL * 60UL * 1000UL; // 10 minutes in ms

String imgBuffer = "";
int expectedChunks = 0;
int receivedChunks = 0;
bool receivingImage = false;

String msgToSend = "";                    // Fila de mensaje de monitor serial
bool msgQueued = false;
unsigned long lastSendAttempt = 0;
unsigned long retryInterval = 500;        // Default LoRa MID: ms entre intervalos de envío

unsigned long ackTimeout = 5000;          // Default LoRa MID: 5 segundo de espera en reintento
unsigned long msgStartTime = 0;           // Cuando el mensaje fue enviado

unsigned long lastChunkRequestTime = 0;
unsigned long chunkTimeout = 3000;        // Default LoRa MID: 3 segundos para reenviar solicitud de chunk

int expectedLength = 128;

// Estados LoRa
enum LoRaRange { SHORT, MID, LONG };
LoRaRange currentRange = SHORT;  // Iniciar SHORT

// RSSI Threshold
const int shortToMid = -85;   // SHORT → MID
const int midToShort  = -45;  // MID → SHORT
const int midToLong   = -102; // MID → LONG
const int longToMid   = -90;  // LONG → MID

// Promedio RSSI
const int rssiSampleCount = 5;    // Number of samples in the rolling average
int rssiSamples[rssiSampleCount] = {0};
int sampleIndex = 0;
int sampleTotal = 0;
int sampleFilled = 0;

void setup() {
  Serial.begin(115200);
  
  while (!Serial);

  dfSerial.begin(9600, SERIAL_8N1, 22, 21);

  if (!myDFPlayer.begin(dfSerial)) {
    while (true);
  }

  // Start AP
  WiFi.softAP(apSSID, apPassword);
  IPAddress IP = WiFi.softAPIP();
  Serial.print("Conectar a Dashboard: ");
  Serial.println(IP);

  // Serve HTML
  server.on("/", HTTP_GET, []() {
    server.send_P(200, "text/html", index_html);
  });

  // ====== Serve JS file ======
  server.on("/main.js", HTTP_GET, []() {
    server.send_P(200, "application/javascript", main_js);
  });

  server.begin();

  // ====== WebSocket server ======
  webSocket.begin();
  webSocket.onEvent(onWebSocketEvent);

  myDFPlayer.volume(23);
  randomSeed(analogRead(0));

  SPI.begin();

  LoRa.setPins(LORA_SS, -1, LORA_DIO0);

  if (!LoRa.begin(LORA_FREQ)) {
    Serial.println("AP_ERR_LoRa_FAILED");
    while (true);
  }

  // LoRa Settings:
  LoRa.setTxPower(20);
  LoRa.setSpreadingFactor(7); // Spreading Factor
  LoRa.setSignalBandwidth(250E3); // BW
  LoRa.setCodingRate4(5); // Coding Rate
  LoRa.setSyncWord(0x88); // Sync word
  LoRa.setPreambleLength(6); // Preamble: 10 symbols
  LoRa.enableCrc(); // CRC

  Serial.println("\nVoyager21: Comunicación LoRa Habillitada");
  Serial.println("'.COMLIST' para Lista de Comandos Disponibles");
  queueTrack(2, 1);
}

void loop() {
  unsigned long now = millis();

  webSocket.loop();
  server.handleClient();

  if (now - lastPunTime >= punInterval) {
    lastPunTime = now;
    playRandomPun();
  }

  if (myDFPlayer.available()) {
    int type = myDFPlayer.readType();
    if (type == DFPlayerPlayFinished) {
      // Shift queue
      for (int i = 1; i < queueSize; i++) trackQueue[i - 1] = trackQueue[i];
      queueSize--;
      if (queueSize > 0)
        myDFPlayer.playFolder(trackQueue[0].folder, trackQueue[0].track);
    }
  }

  // --- RECEIVE: LoRa ---
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    String received = "";
    while (LoRa.available()) received += (char)LoRa.read();
    received.trim();
    
    // Obtener RSSI
    int rssi = LoRa.packetRssi();  // RSSI de mensaje entrante
    sampleTotal -= rssiSamples[sampleIndex];    // Quitar T4
    rssiSamples[sampleIndex] = rssi;            // Guardar T0
    sampleTotal += rssiSamples[sampleIndex];    // Sumar al total

    sampleIndex = (sampleIndex + 1) % rssiSampleCount;    // Circular buffer
    if (sampleFilled < rssiSampleCount) sampleFilled++;

    int avgRssi = sampleTotal / sampleFilled;  // Promediar

    Serial.println("  RECV_ROVER_" + String(rssi) + "," + String(avgRssi) + "," + received);

    if (msgQueued && received.startsWith("ACK_")) {
        String ackFor = received.substring(4);
        if (ackFor == msgToSend) {
            msgQueued = false;
            msgToSend = "";

            if (ackFor == "SRA") LoRaShort();
            if (ackFor == "MRA") LoRaMid();
            if (ackFor == "LRA") LoRaLong();
            if (receivingImage && ackFor.startsWith("REQ_")) lastChunkRequestTime = now;
        }

        handleAckSound(ackFor);
    }

    // Cambio dinámico dependiendo de Thresholds
    switch (currentRange) {
        case SHORT:
            if (avgRssi < shortToMid) {
                Serial.println("AUTOSWITCH_MID");
                queueMessage("MRA");
                queueTrack(2, 12);
            }
            break;

        case MID:
            if (avgRssi > midToShort) {
                Serial.println("AUTOSWITCH_SHORT");
                queueMessage("SRA");
                queueTrack(2, 10);
            } else if (avgRssi < midToLong) {
                Serial.println("AUTOSWITCH_LONG");
                queueMessage("LRA");
                queueTrack(2, 11);
            }
            break;

        case LONG:
            if (avgRssi > longToMid) {
                Serial.println("AUTOSWITCH_MID");
                queueMessage("MRA");
                queueTrack(2, 12);
            }
            break;
    }

    // Handle image ready signal
    if (received.startsWith("IMG_SIZE")) {
      queueSize = 0;
      queueTrack(2, 8);
      queueTrack(1, 1);
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

    else if (received.indexOf(',') != -1) {
      String webTelemetry = String(rssi) + "," + String(avgRssi) + "," + received;
      Serial.println("  RECV_ROVER_" + webTelemetry);
      webSocket.broadcastTXT(webTelemetry);
    }


    // Handle chunk data
    else if (received.startsWith("C_")) {
      if (receivingImage) {
        String chunkData = received.substring(2);

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
            myDFPlayer.playFolder(2, 9);
            // imgBuffer = "";
          }
        } else {
          Serial.printf("CHUNK_%d_INVALID(len=%d,b64=%s)_REREQ\n",
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
      // Send / retry first
      if (now - lastSendAttempt >= retryInterval && LoRa.beginPacket()) {
          LoRa.print(msgToSend);
          LoRa.endPacket();
          lastSendAttempt = now;

          // Start timeout **immediately after first send**
          if (msgStartTime == 0) msgStartTime = now;

          Serial.println("SENT_EST_" + msgToSend);
      }

      // Then check timeout (after potential first send)
      if (msgStartTime > 0 && now - msgStartTime >= ackTimeout) {
          Serial.println("MSG_TIMEOUT");
          msgQueued = false;
          msgToSend = "";
          msgStartTime = 0;
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
  
  // Mensajes Serial
  handleSerial();
}

void handleSerial() {
  while (Serial.available()) {
    String serialInput = Serial.readStringUntil('\n');
    serialInput.trim();

    if (serialInput.length() == 0) continue;

    if (serialInput.equalsIgnoreCase(".COMLIST")) {
      printCommandList();
    }
    else if (serialInput.startsWith(".CHUNK")) {
      String setChunksStr = serialInput.substring(6);
      setChunksStr.trim();
      int setChunks = setChunksStr.toInt();

      if (setChunks > 0 && setChunks < 201) {
        expectedLength = setChunks;
        queueMessage("CK" + String(setChunks));
      } else {
        Serial.println("CHUNK_SIZE_INVALID");
      }
    }
    else if (serialInput.startsWith(".INTERVAL")) {
      String intervalStr = serialInput.substring(9);
      intervalStr.trim();
      float intervalVal = intervalStr.toFloat();

      if (intervalVal > 0) {
        queueMessage("INT" + String(intervalVal));
      } else {
        Serial.println("CSV_INTERVAL_INVALID");
      }
    }
    else if (serialInput.startsWith(".STEP")) {
      String stepsStr = serialInput.substring(5);
      stepsStr.trim();
      int stepsVal = stepsStr.toInt();  // use toInt() for integer steps

      if (stepsVal > 0) {
        queueMessage("SET" + String(stepsVal));
      } else {
        Serial.println("STEP_SIZE_INVALID");
      }
    }

    else if (serialInput.equalsIgnoreCase(".FORCESHORT")) {
      LoRaShort();
    }
    else if (serialInput.equalsIgnoreCase(".FORCEMID")) {
      LoRaMid();
    }
    else if (serialInput.equalsIgnoreCase(".FORCELONG")) {
      LoRaLong();
    }
    else if (serialInput.equalsIgnoreCase(".CANCEL")) {
      // Immediately stop image reception and clear queue
      receivingImage = false;
      imgBuffer = "";
      expectedChunks = 0;
      receivedChunks = 0;
      msgQueued = false;
      msgToSend = "";
      Serial.println("IMAGE_TRANSFER_CANCELLED");
      myDFPlayer.playFolder(2, 13);
    }
    else if (serialInput.equalsIgnoreCase(".W")) queueMessage("W");
    else if (serialInput.equalsIgnoreCase(".S")) queueMessage("S");
    else if (serialInput.equalsIgnoreCase(".A")) queueMessage("A");
    else if (serialInput.equalsIgnoreCase(".D")) queueMessage("D");
    else if (serialInput.equalsIgnoreCase(".IMAGE")) queueMessage("IMG");
    else if (serialInput.equalsIgnoreCase(".SHORT")) queueMessage("SRA");
    else if (serialInput.equalsIgnoreCase(".MID")) queueMessage("MRA");
    else if (serialInput.equalsIgnoreCase(".LONG")) queueMessage("LRA");
    else if (serialInput.equalsIgnoreCase(".START")) queueMessage("GO");
    else if (serialInput.equalsIgnoreCase(".STOP")) queueMessage("STP");
    else if (!handleTrackCommand(serialInput)) {
        Serial.println("UNKNOWN_IGNORED");
    }
  }
}

void queueMessage(String msg) {
  msgToSend = msg;
  msgQueued = true;
  lastSendAttempt = 0;  // force immediate send next loop
  msgStartTime = 0;
}

void LoRaShort() {
  Serial.println("LoRa_CHANGE_SF7");
  LoRa.setSpreadingFactor(7);
  LoRa.setSignalBandwidth(250E3);
  LoRa.setCodingRate4(5);
  LoRa.setPreambleLength(6);
  retryInterval = 300;
  ackTimeout = 3000;
  chunkTimeout = 2000;
  currentRange = SHORT;
}

void LoRaMid() {
  Serial.println("LoRa_CHANGE_SF9");
  LoRa.setSpreadingFactor(9);
  LoRa.setSignalBandwidth(125E3);
  LoRa.setCodingRate4(6);
  LoRa.setPreambleLength(8);
  retryInterval = 500;
  ackTimeout = 5000;
  chunkTimeout = 3000;
  currentRange = MID;
}

void LoRaLong() {
  Serial.println("LoRa_CHANGE_SF11");
  LoRa.setSpreadingFactor(11);
  LoRa.setSignalBandwidth(125E3);
  LoRa.setCodingRate4(8);
  LoRa.setPreambleLength(10);
  retryInterval = 2000;
  ackTimeout = 8000;
  chunkTimeout = 5000;
  currentRange = LONG;
}

void queueTrack(int folder, int track) {
  if (queueSize >= 10) return;
  trackQueue[queueSize++] = {folder, track};

  if (queueSize == 1) {
    myDFPlayer.playFolder(trackQueue[0].folder, trackQueue[0].track);
  }
}

void playRandomPun() {
  int index = random(numPunTracks);
  int trackNum = punTracks[index];
  queueTrack(punFolder, trackNum);
}

void printCommandList() {
  Serial.println("  Lista de Comandos:");
  Serial.println("    '.W'            : Movimiento Hacia Adelante");
  Serial.println("    '.S'            : Movimiento Hacia Atrás");
  Serial.println("    '.A'            : Movimiento CCW");
  Serial.println("    '.D'            : Movimiento CW");
  Serial.println("    '.STEP#'        : Elegir cantidad de Steps (1/32: 6400/Vuelta)");
  Serial.println("    '.INTERVAL#'    : Elegir intervalo de CSV");
  Serial.println("    '.FORCE###'     : Cambiar configuración LoRa: SHORT, MID, LONG");
  Serial.println("    '.CALCULATE'    : Calcula la ruta");
  Serial.println("    '.AUTO'         : Realiza la ruta de manera autonoma");
  Serial.println("    '.REVERSE'      : Realiza la ruta hacia de manera inversa");
  Serial.println("    '.SHOW'         : Muestra la ruta calculada");
  Serial.println("    '.INSTRUCTIONS' : Muestra las instrucciones para realizar la ruta");
  Serial.println("    '.CHANGE'       : Cambiar la meta actual '.CHANGEY,X'");
}

void onWebSocketEvent(uint8_t client_num, WStype_t type, uint8_t * payload, size_t length) {
  switch(type) {
    case WStype_CONNECTED: {
      Serial.printf("CLIENT_%u_CONN\n", client_num);
      // Assign first controller if none
      if(controllerClient == 255) controllerClient = client_num;
      break;
    }
    case WStype_DISCONNECTED: {
      Serial.printf("CLIENT_%u_DC\n", client_num);
      if(client_num == controllerClient) controllerClient = 255;
      break;
    }
    case WStype_TEXT: {
      String msg = String((char*)payload);
      msg.trim();

      if(msg == "WEB_IMG") {
        webSocket.sendTXT(client_num, "IMG_" + imgBuffer); // send current image only to the requester
        return; // skip the controller check
      }

      if(client_num == controllerClient) {
        if(msg == "UP") queueMessage("W");
        else if(msg == "DOWN") queueMessage("S");
        else if(msg == "LEFT") queueMessage("A");
        else if(msg == "RIGHT") queueMessage("D");
        else if(msg == "CAPTURE_IMG") queueMessage("IMG");
        else if(msg == "START_TEL") queueMessage("GO");
        else if(msg == "STOP_TEL") queueMessage("STP");
        else if(msg == "LORA_SHORT") queueMessage("SRA");
        else if(msg == "LORA_MEDIUM") queueMessage("MRA");
        else if(msg == "LORA_LONG") queueMessage("LRA");
      } else {
        Serial.printf("CLIENT_%u_ATTEMPT_NOTCONTROLLER\n", client_num);
      }
      break;
    }
    default:
      break;
  }
}

void handleAckSound(const String &ackFor) {
  if (ackFor == "W") {
    myDFPlayer.playFolder(2, 2);
  } 
  else if (ackFor == "A") {
    myDFPlayer.playFolder(2, 5);
  } 
  else if (ackFor == "S") {
    myDFPlayer.playFolder(2, 3);
  } 
  else if (ackFor == "D") {
    myDFPlayer.playFolder(2, 4);
  } 
  else if (ackFor == "GO") {
    myDFPlayer.playFolder(2, 6);
  } 
  else if (ackFor == "STP") {
    myDFPlayer.playFolder(2, 7);
  }
}

bool handleTrackCommand(const String &cmd) {
    if (cmd.startsWith(".TRACK")) {
        // Extract track number
        int trackNum = cmd.substring(6).toInt();

        if (trackNum <= 0 || trackNum > 255) {
            Serial.println("INVALID_TRACK");
            return true; // we handled it
        }

        // Instead of queueing, play it immediately
        playMusic(1, trackNum); // folder 1, track trackNum

        Serial.printf("PLAYING_TRACK%d\n", trackNum);
        return true; // handled
    }

    return false; // not a .TRACK command
}

void playMusic(int folder, int track) {
    // Optional: stop current music / commands if you want
    myDFPlayer.stop();
    queueSize = 0;           // clear any queued command audio
    myDFPlayer.playFolder(folder, track);
}
