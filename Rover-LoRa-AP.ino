#include <SPI.h>
#include <LoRa.h>
#include <WiFi.h>
#include <WebSocketsServer.h>
#include <WebServer.h>
#include "esp_task_wdt.h"
#define LORA_FREQ 433E6
#define LORA_SS   10    // CS
#define LORA_DIO0 42   // DIO0
#define WDT_TIMEOUT 10   // seconds

//////////////////////////////////////////////////////////////////////////////////////////////////// HTML embebido
const char index_html[] PROGMEM = R"rawliteral(

)rawliteral";

//////////////////////////////////////////////////////////////////////////////////////////////////// JS embebido
const char main_js[] PROGMEM = R"rawliteral(

)rawliteral";

const char* apSSID = "EVA_Dashboard";
const char* apPassword = "Voyager21";  // min 8 chars

WebServer server(80);
WebSocketsServer webSocket = WebSocketsServer(81);

// Only first client can control
uint8_t controllerClient = 255; // 255 = no controller yet

String imgBuffer = "";
String webImage = "";

TaskHandle_t watchdogTaskHandle = NULL;

int expectedChunks = 0;
int receivedChunks = 0;
bool receivingImage = false;

String msgToSend = "";                    // Fila de mensaje de monitor serial
bool msgQueued = false;

unsigned long retryInterval = 500;        // Default LoRa MID: ms entre intervalos de envío
unsigned long ackTimeout = 2000;          // Default LoRa MID: 2 segundo de espera en reintento
unsigned long chunkTimeout = 1500;        // Default LoRa MID: 2 segundos para reenviar solicitud de chunk

unsigned long msgStartTime = 0;           // Cuando el mensaje fue enviado
unsigned long lastSendAttempt = 0;
unsigned long lastChunkRequestTime = 0;

int expectedLength = 128;
float csvInt = 1.5;
int nemaStep = 6400;

// Estados LoRa
enum LoRaRange { SHORT, MID, LONG };
LoRaRange currentRange = SHORT;  // Iniciar SHORT

// RSSI Threshold
const int shortToMid = -65;   // SHORT → MID
const int midToShort  = -45;  // MID → SHORT
const int midToLong   = -100; // MID → LONG
const int longToMid   = -90;  // LONG → MID

// Promedio RSSI
const int rssiSampleCount = 4;    // Number of samples in the rolling average
int rssiSamples[rssiSampleCount] = {0};
int sampleIndex = 0;
int sampleTotal = 0;
int sampleFilled = 0;

void setup() {
  Serial.begin(115200);
  while (!Serial);
  
  const esp_task_wdt_config_t wdt_config = {
        .timeout_ms = WDT_TIMEOUT * 1000,
        .idle_core_mask = (1 << 0) | (1 << 1),
        .trigger_panic = true
  };
  esp_task_wdt_init(&wdt_config);

  esp_task_wdt_add(NULL);
  Serial.println("WATCHDOG_LOOP_ENABLED");

  xTaskCreatePinnedToCore(
    watchdogTask,           // task function
    "watchdogTask",         // name
    4096,                   // stack size
    NULL,                   // parameter
    1,                      // priority
    &watchdogTaskHandle,    // task handle
    0                       // core 0 (low priority)
  );

  // Start AP
  WiFi.softAP(apSSID, apPassword);
  IPAddress IP = WiFi.softAPIP();
  Serial.print("\nConectar a Dashboard: http://");
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

  Serial.println("Voyager21: Comunicación LoRa Habillitada");
  Serial.println("'.COMLIST' para Lista de Comandos Disponibles");
}

void loop() {
  unsigned long now = millis();

  esp_task_wdt_reset();

  webSocket.loop();
  server.handleClient();

  // --- RECEIVE: LoRa ---
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    String received = "";
    while (LoRa.available()) received += (char)LoRa.read();
    received.trim();

    // Filtrar basura
    bool valid = true;
    for (unsigned int i = 0; i < received.length(); i++) {
      char c = received[i];
      if ((c < 32 || c > 126) && c != '\n' && c != '\r') {
        valid = false;
        break;
      }
    }

    if (!valid) {
      Serial.println("NONSENSE_RECEIVED");
      return;  // Descartar antes de actualizar RSSI
    }
    
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
        
      if (ackFor.startsWith("CK")) {
        String lenStr = ackFor.substring(2);
        int newlen = lenStr.toInt();
        if (newlen > 0) expectedLength = newlen;
        Serial.println("  SET_CHUNK_SIZE_" + String(expectedLength));
      }

      if (ackFor.startsWith("CSV_")) {
        String lenStr = ackFor.substring(4);
        float newInt = lenStr.toFloat();
        if (newInt > 0) csvInt = newInt;
      }

      if (ackFor.startsWith("NEMA_")) {
        String lenStr = ackFor.substring(5);
        int newStep = lenStr.toInt();
        if (newStep > 0) nemaStep = newStep;
      }
    }

    // Cambio dinámico dependiendo de Thresholds
    switch (currentRange) {
        case SHORT:
            if (avgRssi < shortToMid) {
                Serial.println("AUTOSWITCH_MID");
                queueMessage("MRA");
            }
            break;

        case MID:
            if (avgRssi > midToShort) {
                Serial.println("AUTOSWITCH_SHORT");
                queueMessage("SRA");
            } else if (avgRssi < midToLong) {
                Serial.println("AUTOSWITCH_LONG");
                queueMessage("LRA");
            }
            break;

        case LONG:
            if (avgRssi > longToMid) {
                Serial.println("AUTOSWITCH_MID");
                queueMessage("MRA");
            }
            break;
    }

    // Handle image ready signal
    if (received.startsWith("IMG_SIZE")) {
      expectedChunks = received.substring(received.indexOf(',') + 1).toInt();
      receivedChunks = 0;
      imgBuffer = "";
      webImage = "";
      receivingImage = true;
      Serial.printf("EXPECTING_%d_CHUNKS\n", expectedChunks);
      webSocket.broadcastTXT("IMG_START");

      // Start requesting first chunk
      msgToSend = "REQ_1";
      msgQueued = true;
      lastSendAttempt = now;
      msgStartTime = now;
      lastChunkRequestTime = now;
    }

    else if (received.indexOf(',') != -1) {
      String webTelemetry = String(rssi) + "," + String(avgRssi) + "," + received;
      webSocket.broadcastTXT(webTelemetry);
    }

    // Handle chunk data
    else if (received.startsWith("C_")) {
      if (receivingImage) {
        String chunkData = received.substring(2);

        bool lastChunk = (receivedChunks == expectedChunks - 1);

        bool b64OK = true;
        for (int i = 0; i < chunkData.length(); i++) {
          char c = chunkData[i];
          if (!((c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') ||
                (c >= '0' && c <= '9') || c == '+' || c == '/' || c == '=')) {
            b64OK = false;
            break;
          }
        }

        bool lengthOK = (lastChunk && chunkData.length() <= expectedLength) ||
                        (chunkData.length() == expectedLength);

        if (lengthOK && b64OK) {
          imgBuffer += chunkData;
          receivedChunks++;
          Serial.printf("  RECV_CHUNK_%d/%d\n", receivedChunks, expectedChunks);

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
            webImage = imgBuffer;
            Serial.println("B64_IMAGE_END");
            imgBuffer = "";
            webSocket.broadcastTXT("IMG_DONE");
          }
        } else {
          Serial.printf("CHUNK_%d_INVALID(len=%d, b64=%s, last=%d)\n",
                        receivedChunks + 1, chunkData.length(),
                        b64OK ? "OK" : "FAIL", lastChunk);
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

    else if (serialInput.startsWith(".FREEZE")) {
      Serial.println("FREEZING...");
      while (true) {
      }
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
      // Detener recepción y limpiar buffer
      cancelImageTransfer();
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
    else if (serialInput.equalsIgnoreCase(".RESET")) queueMessage("RES");
    else if (serialInput.equalsIgnoreCase(".PARTY")) queueMessage("PARTY");
    else  {
      Serial.println("UNKNOWN_IGNORED");
    }
  }
}

void queueMessage(String msg) {
  msgToSend = msg;
  msgQueued = true;
  lastSendAttempt = 0;  // Enviar inmediatamente, enviar en siguiente ciclo si es necesario
  msgStartTime = 0;
}

void broadcastControlState() {
  String msg = "CTRL_" + String(controllerClient);
  webSocket.broadcastTXT(msg);
}

void onWebSocketEvent(uint8_t client_num, WStype_t type, uint8_t * payload, size_t length) {
  switch(type) {
    case WStype_CONNECTED: {
      Serial.printf("CLIENT_%u_CONN\n", client_num);

      // Tell this client its real ESP-assigned ID
      webSocket.sendTXT(client_num, "ASSIGN_ID_" + String(client_num));

      // Send current control state
      String helloMsg = "CTRL_" + String(controllerClient);
      webSocket.sendTXT(client_num, helloMsg);

      // If no controller, make this one controller
      if (controllerClient == 255) {
        controllerClient = client_num;
        broadcastControlState();
        Serial.printf("CLIENT_%u_IS_CONTROLLER\n", client_num);
      }
      break;
    }

    case WStype_DISCONNECTED: {
      Serial.printf("CLIENT_%u_DC\n", client_num);

      // If the controller disconnects, release control
      if (client_num == controllerClient) {
        controllerClient = 255;
        broadcastControlState();
        Serial.println("CONTROLLER_DC_RELEASED");
      }
      break;
    }

    case WStype_TEXT: {
      String msg = String((char*)payload);
      msg.trim();

      // --- Handle control requests ---
      if (msg == "REQUEST_CONTROL") {
        if (controllerClient == 255) {
          controllerClient = client_num;
          broadcastControlState();
          Serial.printf("CLIENT_%u_TAKES_CONTROL\n", client_num);
        } else {
          Serial.printf("CLIENT_%u_REQUEST_DENIED\n", client_num, controllerClient);
        }
        return;
      }

      // --- Handle control release ---
      if (msg == "RELEASE_CONTROL") {
        if (client_num == controllerClient) {
          controllerClient = 255;
          broadcastControlState();
          Serial.printf("CLIENT_%u_RELEASES_CONTROL\n", client_num);
        } else {
          //Serial.printf("CLIENT_%u_NOT_CONTROLLER_CANNOT_RELEASE\n", client_num);
        }
        return;
      }

      // --- Image request (anyone can do this) ---
      if (msg == "WEB_IMG") {
        webSocket.sendTXT(client_num, "IMG_" + webImage);
        return;
      }

      // --- Only controller can send control commands ---
      if (client_num == controllerClient) {
        if (msg == "UP") queueMessage("W");
        else if (msg == "DOWN") queueMessage("S");
        else if (msg == "LEFT") queueMessage("A");
        else if (msg == "RIGHT") queueMessage("D");
        else if (msg == "CAPTURE_IMG") queueMessage("IMG");
        else if (msg == "CANCEL_IMG") cancelImageTransfer();
        else if (msg == "START_TEL") queueMessage("GO");
        else if (msg == "STOP_TEL") queueMessage("STP");
        else if (msg == "LORA_SHORT") LoRaShort();
        else if (msg == "LORA_MEDIUM") LoRaMid();
        else if (msg == "LORA_LONG") LoRaLong();
      } else {
        Serial.printf("CLIENT_%u_ATTEMPTED_CMD\n", client_num);
      }
      break;
    }

    default:
      break;
  }
}

void LoRaShort() {
  Serial.println("LoRa_CHANGE_SF7");
  LoRa.idle();
  delay(150);
  LoRa.setSpreadingFactor(7);
  LoRa.setSignalBandwidth(250E3);
  LoRa.setCodingRate4(5);
  LoRa.setPreambleLength(6);
  delay(50);
  retryInterval = 200;
  ackTimeout = 1000;
  chunkTimeout = 2000;
  currentRange = SHORT;
}

void LoRaMid() {
  Serial.println("LoRa_CHANGE_SF9");
  LoRa.idle();
  delay(150);
  LoRa.setSpreadingFactor(9);
  LoRa.setSignalBandwidth(125E3);
  LoRa.setCodingRate4(6);
  LoRa.setPreambleLength(8);
  delay(50);
  retryInterval = 750;
  ackTimeout = 3000;
  chunkTimeout = 4500;
  currentRange = MID;
}

void LoRaLong() {
  Serial.println("LoRa_CHANGE_SF11");
  LoRa.idle();
  delay(150);
  LoRa.setSpreadingFactor(11);
  LoRa.setSignalBandwidth(125E3);
  LoRa.setCodingRate4(8);
  LoRa.setPreambleLength(10);
  delay(50);
  retryInterval = 1500;
  ackTimeout = 6000;
  chunkTimeout = 5000;
  currentRange = LONG;
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

void cancelImageTransfer() {
  receivingImage = false;
  imgBuffer = "";
  webImage = "";
  expectedChunks = 0;
  receivedChunks = 0;
  msgQueued = false;
  msgToSend = "";
  Serial.println("IMAGE_TRANSFER_CANCELLED");
  webSocket.broadcastTXT("IMG_DONE");
}

void watchdogTask(void *parameter) {
  esp_task_wdt_add(NULL);
  Serial.println("WATCHDOG_TASK_ENABLED");

  while (true) {
    esp_task_wdt_reset();
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}
