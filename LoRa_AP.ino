/////////////////////////////////// EN EL AP, ESTACIÓN TERRESTRE
#include <SPI.h>
#include <LoRa.h>
#include <WiFi.h>

#define LORA_FREQ 433E6

#define LORA_SS   5    // CS
#define LORA_DIO0 25   // DIO0

// WiFi
const char *apSSID = "Voyager21_AP";
const char *apPASS = "12345678";
WiFiServer server(3131);  // TCP Port 12345

bool apEnabled = false; // Bandera: AP state

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

void setup() {
  Serial.begin(115200);
  while (!Serial);

  SPI.begin();

  LoRa.setPins(LORA_SS, -1, LORA_DIO0);

  if (!LoRa.begin(LORA_FREQ)) {
    Serial.println("AP_ERR_LoRa_FAILED");
    while (true);
  }

  // LoRa settings:
  LoRa.setTxPower(20);
  LoRa.setSpreadingFactor(10); // Spreading Factor
  LoRa.setSignalBandwidth(125E3); // BW
  LoRa.setCodingRate4(5); // Coding Rate
  LoRa.setSyncWord(0x88); // Sync word
  LoRa.setPreambleLength(8); // Preamble: 8 symbols
  LoRa.enableCrc(); // CRC
  // ----------------------------

  Serial.println("LORA_READY");
}

WiFiClient client; // global

void loop() {
  // RECV: LoRa
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    String received = "";
    while (LoRa.available()) {
      received += (char)LoRa.read();
    }
    String received_message = "  LORA_STA_RECV_" + received + "_" + LoRa.packetRssi() + "_dBm";
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
      Serial.print("  WIFI_STA_RECV_");
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
    } else if (msg.length() > 0) {
      if (msg.startsWith("W_")) {
        // WiFi
        if (client && client.connected()) {
          String wifiCmd = msg.substring(2); // remove "W_"
          client.println(wifiCmd);
          Serial.print("WIFI_STA_SENT_");
          Serial.println(wifiCmd);
        } else {
          Serial.println("ERROR_NO_WIFI_CLIENT");
        }
      } else {
        // LoRa
        LoRa.beginPacket();
        LoRa.print(msg);
        LoRa.endPacket();

        Serial.print("LORA_AP_SENT_");
        Serial.println(msg);
      }
    }
  }
}
