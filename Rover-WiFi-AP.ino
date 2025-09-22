#include <WiFi.h>
#include <SPI.h>
#include <LoRa.h>

#define LORA_FREQ 433E6
#define LORA_SS   5    // CS
#define LORA_DIO0 25   // DIO0

// Setup de WiFi (AP)
const char* ssid     = "Voyager21";     // Nombre de Red WiFi
const char* password = "Locker31";    // Contraseña

// Configuración TCP
WiFiServer server(3131);   // AP Escucha en Puerto 33  

void setup() {
  Serial.begin(115200);
  delay(500);
  // Inicializar ESP32 en Modo AP
  WiFi.softAP(ssid, password);
  Serial.println("AP Inicializado");
  Serial.print("AP IP address: ");
  Serial.println(WiFi.softAPIP());   // Desplegar IP

  // Iniciar Servidor TCP
  server.begin(3131);
  Serial.println("TCP Server Iniciado en Puerto 3131");
  SPI.begin();

  LoRa.setPins(LORA_SS, -1, LORA_DIO0);

  if (!LoRa.begin(LORA_FREQ)) {
    Serial.println("LoRa init failed!");
    while (true);
  }

  // --- Robustness settings ---
  LoRa.setTxPower(20);             // max TX power (17–20 typical for SX1276)
  LoRa.setSpreadingFactor(12);     // 6–12, higher = more robust, slower
  LoRa.setSignalBandwidth(125E3);  // 62.5E3 or 125E3 good for robustness
  LoRa.setCodingRate4(8);          // 5–8, higher = more robust
  // ----------------------------

  Serial.println("LoRa Inicializado");
}

WiFiClient client;  // Declare once, outside loop()

void loop() {
  // --------- WiFi STA Connection Handling ---------
  if (!client || !client.connected()) {
    client = server.available();  // Accept new connection if none
    if (client) {
      Serial.println("STA Conectado");
      Serial.println("'.COMLIST' para Lista de Comandos Disponibles");
    }
  }

  if (client && client.connected()) {
    // --- 1a. Check messages from STA (WiFi) ---
    if (client.available()) {
      String msg = client.readStringUntil('\n');
      msg.trim();
      if (msg.length() > 0) {
        Serial.print("RECV_STA_");
        Serial.println(msg);
      }
    }
  }

  // --------- Serial Input Handling ---------
  if (Serial.available()) {
    String userMsg = Serial.readStringUntil('\n');
    userMsg.trim();

    if (userMsg.length() > 0) {
      // --- LoRa Message ---
      if (userMsg.startsWith("lora.")) {
        LoRa.beginPacket();
        LoRa.print(userMsg);  // Keep "lora." prefix
        LoRa.endPacket();
        Serial.print("LORA_AP_SENT_");
        Serial.println(userMsg);
      } 
      // --- WiFi Message ---
      else if (client && client.connected()) {
        if (userMsg.equalsIgnoreCase(".COMLIST")) {
          Serial.println(userMsg);
          Serial.println("  Lista de Comandos:");
          Serial.println("    '.RSSI'   : Intensidad de Señal Recibida");
          Serial.println("    '.AMBTEMP': Lectura de Temperatura y Humedad (Ambiente)");
          Serial.println("    '.INTTEMP': Lectura de Temperatura y Humedad (Interna)");
          Serial.println("    '.GYRO'   : Lectura de Acelerómetro y Giroscopio");
          Serial.println("    '.POWER#' : V,I,Pow ; # = 1 (ESP), 2 (Motor 1), 3 (Motor 3)");
          Serial.println("    '.DIST#'  : Distancia de Sensor de Laser # = 1, 2, o 3");
          Serial.println("    '.SET#'   : Selección de Pasos (200 = 1 revolución)");
          Serial.println("    '.W'      : Movimiento Hacia Adelante");
          Serial.println("    '.S'      : Movimiento Hacia Detrás");
          Serial.println("    '.A'      : Movimiento Hacia Izquierda");
          Serial.println("    '.D'      : Movimiento Hacia Derecha");
        } else {
          client.println(userMsg);  // Send to STA
          Serial.print("SENT_AP_");  // Log
          Serial.println(userMsg);
        }
      }
    }
  }

  // --------- LoRa Handling ---------
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    String received = "";
    while (LoRa.available()) {
      received += (char)LoRa.read();
    }
    Serial.print("LORA_STA_RECV_");
    Serial.print(received);
    Serial.print("_");
    Serial.print(LoRa.packetRssi());
    Serial.println("_dBm");
  }

  delay(50);
}