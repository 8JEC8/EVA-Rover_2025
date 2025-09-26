#include "esp_camera.h"
#include "FS.h"
#include "SPIFFS.h"

// ===================== PINOUT AI-THINKER =====================
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
  #define LED_FLASH_GPIO     4
#endif

// ===================== PARÁMETROS DE CAPTURA =================
static framesize_t FRAME_SIZE_PRIMARY = FRAMESIZE_QVGA; // 320x240
static int         JPEG_QUALITY       = 40;             // más alto = más compresión

// ===================== BASE64 (encoder) ======================
static const char B64T[] =
  "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

// Chunks de Base64 separados por delimitador (7 espacios por defecto)
void encodeBase64ChunkedDelimited(const uint8_t* data, size_t len, Print& out,
                                  size_t chunkChars = 128, const char* delim = "       ") {
  if (chunkChars < 4) chunkChars = 4;
  if (chunkChars % 4 != 0) chunkChars -= (chunkChars % 4);

  size_t i = 0;
  size_t line = 0;
  bool firstChunk = true;

  auto writeQuartet = [&](uint32_t n) {
    if (line == 0 && !firstChunk) out.print(delim);
    out.write(B64T[(n >> 18) & 63]);
    out.write(B64T[(n >> 12) & 63]);
    out.write(B64T[(n >> 6)  & 63]);
    out.write(B64T[n & 63]);
    line += 4;
    if (line == chunkChars) { line = 0; firstChunk = false; }
  };

  while (i + 2 < len) {
    uint32_t n = (uint32_t)data[i] << 16 | (uint32_t)data[i+1] << 8 | (uint32_t)data[i+2];
    writeQuartet(n);
    i += 3;
  }

  if (i < len) {
    if (line == 0 && !firstChunk) out.print(delim);
    uint32_t n = (uint32_t)data[i] << 16;
    out.write(B64T[(n >> 18) & 63]);
    out.write(B64T[(n >> 12) & 63]);
    if (i + 1 < len) {
      n |= (uint32_t)data[i+1] << 8;
      out.write(B64T[(n >> 6) & 63]);
      out.write('=');
    } else {
      out.write('=');
      out.write('=');
    }
  }
  out.write('\n'); // fin de la línea de chunks
}

// Una sola línea (sin separadores)
void encodeBase64OneLine(const uint8_t* data, size_t len, Print& out) {
  size_t i = 0;
  while (i + 2 < len) {
    uint32_t n = (uint32_t)data[i] << 16 | (uint32_t)data[i+1] << 8 | (uint32_t)data[i+2];
    out.write(B64T[(n >> 18) & 63]);
    out.write(B64T[(n >> 12) & 63]);
    out.write(B64T[(n >> 6)  & 63]);
    out.write(B64T[n & 63]);
    i += 3;
  }
  if (i < len) {
    uint32_t n = (uint32_t)data[i] << 16;
    out.write(B64T[(n >> 18) & 63]);
    out.write(B64T[(n >> 12) & 63]);
    if (i + 1 < len) {
      n |= (uint32_t)data[i+1] << 8;
      out.write(B64T[(n >> 6) & 63]);
      out.write('=');
    } else {
      out.write('=');
      out.write('=');
    }
  }
}

// ===================== CÁMARA ================================
bool initCam(bool usePsram, framesize_t fs, int xclk_hz, int fb_loc) {
  camera_config_t c{};
  c.ledc_channel = LEDC_CHANNEL_0;
  c.ledc_timer   = LEDC_TIMER_0;
  c.pin_d0       = Y2_GPIO_NUM;
  c.pin_d1       = Y3_GPIO_NUM;
  c.pin_d2       = Y4_GPIO_NUM;
  c.pin_d3       = Y5_GPIO_NUM;
  c.pin_d4       = Y6_GPIO_NUM;
  c.pin_d5       = Y7_GPIO_NUM;
  c.pin_d6       = Y8_GPIO_NUM;
  c.pin_d7       = Y9_GPIO_NUM;
  c.pin_xclk     = XCLK_GPIO_NUM;
  c.pin_pclk     = PCLK_GPIO_NUM;
  c.pin_vsync    = VSYNC_GPIO_NUM;
  c.pin_href     = HREF_GPIO_NUM;
  c.pin_sscb_sda = SIOD_GPIO_NUM;
  c.pin_sscb_scl = SIOC_GPIO_NUM;
  c.pin_pwdn     = PWDN_GPIO_NUM;
  c.pin_reset    = RESET_GPIO_NUM;

  c.xclk_freq_hz = xclk_hz;
  c.pixel_format = PIXFORMAT_JPEG;
  c.frame_size   = fs;
  c.jpeg_quality = JPEG_QUALITY;
  c.fb_count     = usePsram ? 2 : 1;
  c.grab_mode    = CAMERA_GRAB_WHEN_EMPTY;
  c.fb_location  = (camera_fb_location_t)fb_loc;

  esp_err_t err = esp_camera_init(&c);
  if (err != ESP_OK) {
    Serial.printf("esp_camera_init fallo (0x%x)\n", err);
    return false;
  }
  return true;
}

// ===================== SETUP =================================
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\nESP32-CAM: JPG a SPIFFS + Base64 (chunks 128 y todo junto)");

  if (!SPIFFS.begin(true)) {
    Serial.println("ERROR: no se pudo montar SPIFFS");
    while (true) delay(1000);
  }

  pinMode(LED_FLASH_GPIO, OUTPUT);
  digitalWrite(LED_FLASH_GPIO, LOW);

  bool ok = false;
  bool psram = psramFound();
  Serial.printf("PSRAM %s\n", psram ? "detectada" : "NO detectada");

  if (psram) ok = initCam(true, FRAME_SIZE_PRIMARY, 20000000, CAMERA_FB_IN_PSRAM);
  if (!ok && psram) {
    Serial.println("Reintento: PSRAM + XCLK 10MHz");
    ok = initCam(true, FRAME_SIZE_PRIMARY, 10000000, CAMERA_FB_IN_PSRAM);
  }
  if (!ok) {
    Serial.println("Reintento: DRAM + QQVGA + XCLK 10MHz + fb_count=1");
    ok = initCam(false, FRAMESIZE_QQVGA, 10000000, CAMERA_FB_IN_DRAM);
  }
  if (!ok) { Serial.println("ERROR: No se pudo iniciar la camara."); while (true) delay(1000); }

  // Captura
  Serial.println("Capturando foto...");
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) { Serial.println("ERROR: esp_camera_fb_get devolvio NULL"); while (true) delay(1000); }
  Serial.printf("JPEG len = %u bytes\n", (unsigned)fb->len);

  // Guardar JPG
  const char *jpgPath = "/photo.jpg";
  {
    File f = SPIFFS.open(jpgPath, FILE_WRITE);
    if (!f) { Serial.println("ERROR: no se pudo abrir /photo.jpg"); esp_camera_fb_return(fb); while (true) delay(1000); }
    size_t w = f.write(fb->buf, fb->len);
    f.close();
    Serial.printf("JPG guardado %s (%u bytes)\n", jpgPath, (unsigned)w);
  }

  // -------------------------------------------
  // 1) CHUNKS (128 chars) separados por 7 espacios
  // -------------------------------------------
  Serial.println("-----BEGIN JPEG BASE64 CHUNKS-----");
  encodeBase64ChunkedDelimited(fb->buf, fb->len, Serial, 128, "       "); // 7 espacios
  Serial.println("-----END JPEG BASE64 CHUNKS-----");

  // -------------------------------------------
  // 2) TODO JUNTO (misma imagen, una sola línea)
  // -------------------------------------------
  Serial.println("-----BEGIN JPEG BASE64 FULL-----");
  encodeBase64OneLine(fb->buf, fb->len, Serial);
  Serial.println();
  Serial.println("-----END JPEG BASE64 FULL-----");

  // (opcional) Guardar la versión en 1 línea en SPIFFS
  const char *b64Path = "/photo.b64";
  {
    File b64 = SPIFFS.open(b64Path, FILE_WRITE);
    if (!b64) {
      Serial.println("ERROR: no se pudo abrir /photo.b64");
    } else {
      encodeBase64OneLine(fb->buf, fb->len, b64);
      b64.write('\n');
      b64.close();
      Serial.printf("Base64 guardado %s\n", b64Path);
    }
  }

  esp_camera_fb_return(fb);

  Serial.println("Listo. Arriba están los CHUNKS y luego la cadena FULL (todo junto).");
  Serial.println("Para navegador: data:image/jpeg;base64,<PEGA_CADENA_FULL>");
}

void loop() {
  delay(5000);
}