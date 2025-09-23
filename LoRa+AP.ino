/////////////////////////////////// EN EL AP, ESTACIÓN TERRESTRE
#include <SPI.h>
#include <LoRa.h>

#define LORA_FREQ 433E6

// LoRa pins for Devkit V1
#define LORA_SS   5    // CS
#define LORA_DIO0 25   // DIO0


void setup() {
  Serial.begin(115200);
  while (!Serial);

  Serial.println("=== LoRa Chat ===");
  Serial.println("Type a message and press Enter to send.\n");

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

  Serial.println("LoRa initialized. Waiting for messages...");
}

void loop() {
  // 1. Check RECV
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    String received = "";
    while (LoRa.available()) {
      received += (char)LoRa.read();
    }
    String received_message = "LORA_STA_RECV_" + received + "_" + LoRa.packetRssi() + "_dBm";
    Serial.println(received_message);
  }

  // 2. Check SerialMonitor
  if (Serial.available()) {
    String msg = Serial.readStringUntil('\n');
    msg.trim();

    if (msg.length() > 0) {
      LoRa.beginPacket();
      LoRa.print(msg);
      LoRa.endPacket();

      Serial.print("LORA_AP_SENT_");
      Serial.println(msg);
    }
  }
}