#include <SPI.h>
#include <LoRa.h>
#include "esp_camera.h"
#include "base64.h"   // Arduino Base64 library
#include <vector>

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

// Globals
std::vector<String> imageChunks;
int totalChunks = 0;
int chunkSize = 128;

bool receivingTel = false;   // Already exists
bool wasReceivingTel = false; // NEW flag to remember previous state

// Camera Init
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
    Serial.printf("ERR_CAMERA_INIT0x%x", err);
    while (true);
  }
}

// --- Capture and store image ---
void captureAndStoreImage() {
  // clear previous image
  imageChunks.clear();
  totalChunks = 0;

  // flush old buffer
  camera_fb_t *fb_old = esp_camera_fb_get();
  if (fb_old) esp_camera_fb_return(fb_old);

  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("CAPTURE_FAILED");
    return;
  }

  // Encode to Base64
  String encoded = base64::encode(fb->buf, fb->len);
  esp_camera_fb_return(fb);

  int totalLen = encoded.length();
  totalChunks = (totalLen + chunkSize - 1) / chunkSize;

  for (int i = 0; i < totalChunks; i++) {
    int start = i * chunkSize;
    int end = min(start + chunkSize, totalLen);
    imageChunks.push_back(encoded.substring(start, end));
  }

  // Inform receiver
  LoRa.beginPacket();
  LoRa.print("IMG_SIZE," + String(totalChunks));
  LoRa.endPacket();
}

void setup() {
  Serial.begin(115200);
  while (!Serial);

  // LoRa setup
  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_SS);
  LoRa.setPins(LORA_SS, -1, LORA_DIO0);

  if (!LoRa.begin(LORA_FREQ)) {
    Serial.println("LoRa_ERROR_init");
    while (true);
  }

  LoRa.setTxPower(20);
  LoRa.setSpreadingFactor(7);
  LoRa.setSignalBandwidth(250E3);
  LoRa.setCodingRate4(5);
  LoRa.setSyncWord(0x88);
  LoRa.setPreambleLength(6);
  LoRa.enableCrc();

  initCamera();
}

void loop() {
  // LoRa check
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

    if (received == "SRA" || received == "MRA" || received == "LRA") {
      delay(100);  // 100 para acabar ACK
    }

    handleRequest(received);
  }

  // Rover (RECV Serial) a Estación (SENT LoRa)
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

void handleRequest(String cmd) {
  if (cmd == "GO") {
    receivingTel = true;
  }

  if (cmd == "STP") {
    receivingTel = false;
  }

  if (cmd.startsWith("CK")) {
      String numStr = cmd.substring(2);
      int newSize = numStr.toInt();
      chunkSize = newSize;
      LoRa.beginPacket();
      LoRa.print("SET_CHUNKS_" + String(chunkSize));
      LoRa.endPacket();
  }

  if (cmd == "SRA") {
    LoRa.setSpreadingFactor(7);
    LoRa.setSignalBandwidth(250E3);
    LoRa.setCodingRate4(5);
    LoRa.setPreambleLength(6);
    Serial.println("INT1.0");
  }

  if (cmd == "MRA") {
    LoRa.setSpreadingFactor(9);
    LoRa.setSignalBandwidth(125E3);
    LoRa.setCodingRate4(6);
    LoRa.setPreambleLength(8);
    Serial.println("INT2.0");
  }

  if (cmd == "LRA") {
    LoRa.setSpreadingFactor(11);
    LoRa.setSignalBandwidth(125E3);
    LoRa.setCodingRate4(8);
    LoRa.setPreambleLength(10);
    Serial.println("INT8.0");
  }

  if (cmd == "IMG") {
    wasReceivingTel = receivingTel;

    // Detener TEL y capturar Imagen + Flash LED
    Serial.println("STP");
    receivingTel = false;
    Serial.println("FCAM");
    captureAndStoreImage();
    return;
  }

  if (cmd.startsWith("REQ_")) {
      int reqNum = cmd.substring(4).toInt();
      if (reqNum >= 1 && reqNum <= totalChunks) {

          if (reqNum == totalChunks) {
              delay(250);
          }

          String packet = "C_" + imageChunks[reqNum - 1];

          LoRa.beginPacket();
          LoRa.print(packet);
          LoRa.endPacket();
          delay(25); // small gap between packets

          // Resume TEL after sending last chunk
          if (reqNum == totalChunks && wasReceivingTel) {
              Serial.println("GO");
              receivingTel = true;
          }

      } else {
          // Número de Chunk no válido
      }
      return;
  }

  // FWD LoRa a Rover a través de Serial
  Serial.println(cmd);
}
