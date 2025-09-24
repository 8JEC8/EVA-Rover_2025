#include <SPI.h>
#include <LoRa.h>
#include <Adafruit_NeoPixel.h>

#define LORA_FREQ 433E6

// LoRa pins for ESP32-CAM
#define LORA_SCK   14   // SCK
#define LORA_MISO  12   // MISO
#define LORA_MOSI  13   // MOSI
#define LORA_SS    15   // CS (NSS)
#define LORA_DIO0   4   // DIO0

// WS2812 LED strip
#define LED_PIN    16     // choose a safe GPIO (2 or 16 recommended)
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

  // Initialize SPI with custom pins for ESP32-CAM
  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_SS);

  // Attach LoRa with defined CS and DIO0 pins
  LoRa.setPins(LORA_SS, -1, LORA_DIO0);

  if (!LoRa.begin(LORA_FREQ)) {
    Serial.println("LoRa init failed!");
    while (true);
  }

  // LoRa
  LoRa.setTxPower(20);             
  LoRa.setSpreadingFactor(12);     
  LoRa.setSignalBandwidth(125E3);  
  LoRa.setCodingRate4(8);          

  // Iniciar LEDs
  strip.begin();
  strip.show(); // Apagar LEDs
}

void loop() {
  // 1. Check RECV
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    String received = "";
    while (LoRa.available()) {
      received += (char)LoRa.read();
    }
    
    String ack = "ACK_" + received;
    LoRa.beginPacket();
    LoRa.print(ack);
    LoRa.endPacket();

    if (received == ".RED") {
      setColor(10, 0, 0);
    } else if (received == ".GREEN") {
      setColor(0, 10, 0);
    } else if (received == ".BLUE") {
      setColor(0, 0, 10);
    } else if (received == ".YELLOW") {
      setColor(10, 10, 0);
    } else if (received == ".CYAN") {
      setColor(0, 10, 10);
    } else if (received == ".MAGENTA") {
      setColor(10, 0, 10);
    } else if (received == ".WHITE") {
      setColor(10, 10, 10);
    } else if (received == ".ORANGE") {
      setColor(10, 4, 0);
    } else if (received == ".PURPLE") {
      setColor(5, 0, 10);
    } else if (received == ".OFF") {
      setColor(0, 0, 0);
    }
  }
}