#include <SPI.h>
#include <LoRa.h>
#include <Adafruit_NeoPixel.h>

#define LORA_FREQ 433E6

// LoRa pins
#define LORA_SCK   14
#define LORA_MISO  12
#define LORA_MOSI  13
#define LORA_SS    15
#define LORA_DIO0   4

// WS2812: LED strip
#define LED_PIN    16
#define NUM_LEDS   8
Adafruit_NeoPixel strip(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);

void setColor(uint8_t r, uint8_t g, uint8_t b) {
  for (int i = 0; i < NUM_LEDS; i++) {
    strip.setPixelColor(i, strip.Color(r, g, b));
  }
  strip.show();
}

void setup() {
  Serial.begin(115200);
  while (!Serial);

  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_SS);
  LoRa.setPins(LORA_SS, -1, LORA_DIO0);

  if (!LoRa.begin(LORA_FREQ)) {
    Serial.println("LoRa init failed!");
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

  strip.begin();
  strip.show(); // LED OFF
}

void loop() {
  // LoRa check, forward to serial
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    String received = "";
    while (LoRa.available()) {
      received += (char)LoRa.read();
    }
    received.trim();

    // Ack, LoRa
    LoRa.beginPacket();
    LoRa.print("ACK_" + received);
    LoRa.endPacket();


    if (received == ".RED") setColor(10,0,0);
    else if (received == ".GREEN") setColor(0,10,0);
    else if (received == ".BLUE") setColor(0,0,10);
    else if (received == ".YELLOW") setColor(10,10,0);
    else if (received == ".CYAN") setColor(0,10,10);
    else if (received == ".MAGENTA") setColor(10,0,10);
    else if (received == ".WHITE") setColor(10,10,10);
    else if (received == ".ORANGE") setColor(10,4,0);
    else if (received == ".PURPLE") setColor(5,0,10);
    else if (received == ".OFF") setColor(0,0,0);

    else {
      Serial.println(received);
    }
  }

  // Serial check, forward to LoRa
  while (Serial.available()) {
    String staResponse = Serial.readStringUntil('\n');
    staResponse.trim();
    if (staResponse.length() > 0) {
      LoRa.beginPacket();
      LoRa.print(staResponse);
      LoRa.endPacket();
    }
  }
}
