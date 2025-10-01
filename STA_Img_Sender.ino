#include <SPI.h>
#include <LoRa.h>
#include "esp_camera.h"
#include "base64.h"   // Arduino Base64 library

#define LORA_FREQ 433E6

// LoRa pins
#define LORA_SCK   14
#define LORA_MISO  12
#define LORA_MOSI  13
#define LORA_SS    15
#define LORA_DIO0   4

// ESP32-CAM pin mapping
#define CAMERA_MODEL_AI_THINKER
#if defined(CAMERA_MODEL_AI_THINKER)
  #define PWDN_GPIO_NUM     32
  #define RESET_GPIO_NUM    -1
  #define XCLK_GPIO_NUM      0
  #define SIOD_GPIO_NUM     26
  #define SIOC_GPIO_NUM     27
  #define Y9_GPIO_NUM       35
  #define Y8_GPIO_NUM       34
  #define Y7_GPIO_NUM       39
  #define Y6_GPIO_NUM       36
  #define Y5_GPIO_NUM       21
  #define Y4_GPIO_NUM       19
  #define Y3_GPIO_NUM       18
  #define Y2_GPIO_NUM        5
  #define VSYNC_GPIO_NUM    25
  #define HREF_GPIO_NUM     23
  #define PCLK_GPIO_NUM     22
  #define LED_FLASH_GPIO    -1
#endif

// --- Initialize camera ---
void initCamera() {
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;

  if (psramFound()) {
    config.frame_size = FRAMESIZE_QVGA;  // ~320x240
    config.jpeg_quality = 10;
    config.fb_count = 1;
  }

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    while (true);
  }
}

// --- Capture, encode, and send in chunks ---
void captureAndSendImage() {
  camera_fb_t *fb_old = esp_camera_fb_get();  // grab current buffer
  if (fb_old) esp_camera_fb_return(fb_old);   // immediately release it

  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("Camera capture failed");
    return;
  }

  // Encode to Base64
  String encoded = base64::encode(fb->buf, fb->len);
  esp_camera_fb_return(fb);

  int totalLen = encoded.length();
  int chunkSize = 128;
  int totalChunks = (totalLen + chunkSize - 1) / chunkSize;

  Serial.printf("Sending image: %d bytes base64, %d chunks\n", totalLen, totalChunks);

  for (int i = 0; i < totalChunks; i++) {
    int start = i * chunkSize;
    int end = min(start + chunkSize, totalLen);
    String chunk = encoded.substring(start, end);

    // Format: IMG_<chunk>/<total>,<data>
    String packet = "IMG_" + String(i + 1) + "/" + String(totalChunks) + "," + chunk;

    LoRa.beginPacket();
    LoRa.print(packet);
    LoRa.endPacket();

    delay(200); // small delay to avoid flooding LoRa
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial);

  // LoRa
  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_SS);
  LoRa.setPins(LORA_SS, -1, LORA_DIO0);

  if (!LoRa.begin(LORA_FREQ)) {
    Serial.println("LoRa init failed!");
    while (true);
  }
  
  LoRa.setTxPower(20);
  LoRa.setSpreadingFactor(10);
  LoRa.setSignalBandwidth(125E3);
  LoRa.setCodingRate4(5);
  LoRa.setSyncWord(0x88);
  LoRa.setPreambleLength(8);
  LoRa.enableCrc();

  // Camera
  initCamera();
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

    // Ack
    LoRa.beginPacket();
    LoRa.print("ACK_" + received);
    LoRa.endPacket();

    if (received == ".IMAGE") {
      captureAndSendImage();
    } else {
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
