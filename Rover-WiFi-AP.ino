#include <SPI.h>
#include <LoRa.h>
#include <WiFi.h>

#define LORA_FREQ 433E6

#define LORA_SS   5    // CS
#define LORA_DIO0 25   // DIO0

// Setup de WiFi (AP)
const char* apSSID     = "Voyager21";     // Nombre de Red WiFi
const char* apPASS = "Locker31";    // Contraseña
bool apEnabled = false; // Bandera: AP state
WiFiServer server(3131);   // TCP: AP Escucha en Puerto 33
WiFiClient client;  // Global

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
  LoRa.setSpreadingFactor(10); // Spreading Factor
  LoRa.setSignalBandwidth(125E3); // BW
  LoRa.setCodingRate4(5); // Coding Rate
  LoRa.setSyncWord(0x88); // Sync word
  LoRa.setPreambleLength(8); // Preamble: 8 symbols
  LoRa.enableCrc(); // CRC

  Serial.println("\n");
  Serial.println("Voyager21: LoRa Communication Ready");
  Serial.println("'.COMLIST' para Lista de Comandos Disponibles");
}

void loop() {
  // RECV: LoRa
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    String received = "";
    while (LoRa.available()) {
      received += (char)LoRa.read();
    }
    String received_message = "  LORA_RECV_STA_" + received + "_" + LoRa.packetRssi() + "_dBm";
    Serial.println(received_message);
  }

  // Accept new clients
  WiFiClient newClient = server.available();
  if (newClient) {
    if (client && client.connected()) {
      newClient.stop();  // Only allow one client at a time
    } else {
      client = newClient;
      Serial.println(" AP_NEW_CLIENT_CONNECTED");
    }
  }

  // RECV: WiFi
  if (client && client.connected() && client.available()) {
    String wifiReceived = client.readStringUntil('\n');
    wifiReceived.trim();
    if (wifiReceived.length() > 0) {
      Serial.print("  WIFI_RECV_STA_");
      Serial.println(wifiReceived);
    }
  }

  // SENT: Serial STA (WiFi + LoRa)
  if (Serial.available()) {
    String msg = Serial.readStringUntil('\n');
    msg.trim();

    if (msg.equalsIgnoreCase(".APON")) {
      Serial.println(msg);
      startAP();
    } else if (msg.equalsIgnoreCase(".APOFF")) {
      Serial.println(msg);
      stopAP();
    } else if (msg.equalsIgnoreCase(".COMLIST")){
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
    } else if (msg.length() > 0) {
      if (msg.startsWith("W_")) {
        // WiFi
        if (client && client.connected()) {
          String wifiCmd = msg.substring(2); // remove "W_"
          client.println(wifiCmd);
          Serial.print("WIFI_SENT_AP_");
          Serial.println(wifiCmd);
        } else {
          Serial.println("ERROR_NO_WIFI_CLIENT");
        }
      } else {
        // LoRa
        LoRa.beginPacket();
        LoRa.print(msg);
        LoRa.endPacket();

        Serial.print("LORA_SENT_AP_");
        Serial.println(msg);
      }
    }
  }
}

void startAP() {
  if (!apEnabled) {
    WiFi.disconnect(true); // Clear session
    WiFi.mode(WIFI_OFF);
    delay(100);            // Delay WiFi driver

    WiFi.mode(WIFI_AP);
    bool result = WiFi.softAP(apSSID, apPASS);
    if (result) {
      Serial.println(" AP_WiFi_ENABLED");
      Serial.print(" SSID: ");
      Serial.println(apSSID);
      Serial.print(" IP: ");
      Serial.println(WiFi.softAPIP());
      server.begin();
      server.setNoDelay(true);
      apEnabled = true;
    } else {
      Serial.println(" AP_ERR_AP_SETUP");
    }
  } else {
    Serial.println(" AP_WiFi_RUNNING");
  }
}

void stopAP() {
  if (apEnabled) {
    uint8_t numClients = WiFi.softAPgetStationNum();
    if (numClients > 0) {
      Serial.println("LORA_AP_.APOFF");
      Serial.printf(" AP_DCING_%d_CLIENTS\n", numClients);
      WiFi.softAPdisconnect(false); // DC clients
      delay(100); // Process DC
    }

    WiFi.softAPdisconnect(true); // Stop AP
    WiFi.mode(WIFI_OFF);
    delay(100); // Delay: WiFi Driver
    Serial.println(" AP_WiFi_DISABLED");
    apEnabled = false;
  } else {
    Serial.println(" AP_WiFi_NOTRUNNING");
  }
}
