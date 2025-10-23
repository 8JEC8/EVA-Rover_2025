#include <Wire.h>
#include <Arduino.h>
#include <Adafruit_INA219.h>
#include <Adafruit_SHT31.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_AHTX0.h>
#include <VL53L0X.h>
#include <Adafruit_NeoPixel.h>

VL53L0X sensor1;
VL53L0X sensor2;
VL53L0X sensor3;
Adafruit_INA219 ina219_ESP(0x40);
Adafruit_INA219 ina219_M1(0x41);
Adafruit_INA219 ina219_M2(0x45);
Adafruit_SHT31 sht31 = Adafruit_SHT31();
Adafruit_MPU6050 mpu;
Adafruit_AHTX0 aht;

#define DIR1_PIN 27   //Motor1 and Motor2 Pins
#define STEP1_PIN 26
#define SLEEP_PIN 25
#define DIR2_PIN 18
#define STEP2_PIN 19

#define XSHUT1 16     //XSHUT para VL53L0X addresses
#define XSHUT2 17
#define XSHUT3 4
#define RX_PIN 3   // UART RX desde Serial entrando de LoRa
#define TX_PIN 1   // UART TX para Serial FWD LoRa

#define LED_PIN 5
#define LED_COUNT 8
Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

int stepsPerRev = 6400;    // 1 revolución por defecto, cambiar con .STEP###
#define STEP_DELAY_US 63   // Velocidad

bool ledBreathing = false;
bool ledFlashing = false;
bool ledRainbow = false;
bool ledDot = false;
bool ledIdle = true;
bool ledCam = false; 

unsigned long previousMillis = 0;
float intervalSec = 2.5;                      // interval in seconds (default)
unsigned long interval = intervalSec * 1000;  // converted to ms
bool sendCSV = false;                         // Flag to control CSV sending

#define CSV_BUFFER_SIZE 96
char csvBuffer[CSV_BUFFER_SIZE];

enum MotorCommand {
  MOTOR_IDLE,
  MOTOR_BOTH_CW,
  MOTOR_BOTH_CCW,
  MOTOR_OPPOSITE_A,
  MOTOR_OPPOSITE_D
};

volatile MotorCommand currentCommand = MOTOR_IDLE;

void setup() {
  Serial.begin(115200);
  Serial.setDebugOutput(false);

  strip.begin();  // Inicializar strip
  strip.show();   // Apagar LEDs

  pinMode(DIR2_PIN, OUTPUT);
  pinMode(STEP2_PIN, OUTPUT);
  pinMode(SLEEP_PIN, OUTPUT);
  pinMode(DIR1_PIN, OUTPUT);
  pinMode(STEP1_PIN, OUTPUT);

  digitalWrite(SLEEP_PIN, LOW); // Sleep motores inmediatamente
  
  xTaskCreatePinnedToCore(
    motorTask,       
    "MotorTask",     
    4096,            
    NULL,            
    1,               
    NULL,            
    1                // core 1
  );

  xTaskCreatePinnedToCore(
    ledTask,        // Task function
    "LED Task",     // Name
    2048,           // Stack size
    NULL,           // Parameter
    1,              // Priority
    NULL,           // Task handle
    1               // Run on core 1
  );

  pinMode(XSHUT1, OUTPUT);
  pinMode(XSHUT2, OUTPUT);
  pinMode(XSHUT3, OUTPUT);

  Wire.begin(); // I2C sensores de distancia

  // Apagamos todos los sensores de distancia previo a escribir Address
  digitalWrite(XSHUT1, LOW);
  digitalWrite(XSHUT2, LOW);
  digitalWrite(XSHUT3, LOW);
  delay(20);

  // Inicializamos sensor 1
  digitalWrite(XSHUT1, HIGH);
  delay(20);
  sensor1.init(true);
  sensor1.setAddress(0x30);   // Dirección temporal para sensor 1
  sensor1.startContinuous();

  // Inicializamos sensor 2
  digitalWrite(XSHUT2, HIGH);
  delay(20);
  sensor2.init(true);
  sensor2.setAddress(0x31);   // Dirección temporal para sensor 2
  sensor2.startContinuous();

  // Inicializamos sensor 3
  digitalWrite(XSHUT3, HIGH);
  delay(20);
  sensor3.init(true);
  sensor3.setAddress(0x32);   // Dirección temporal para sensor 3
  sensor3.startContinuous();

  while (!ina219_ESP.begin()) {
    Serial.println("ERROR_INA_ESP32");
    delay(20);
  }

  while (!ina219_M1.begin()) {
    Serial.println("ERROR_INA_MOTOR1");
    delay(20);
  }

  while (!ina219_M2.begin()) {
    Serial.println("ERROR_INA_MOTOR2");
    delay(20);
  }

  // Intento de Conexión: SHT31
  while (!sht31.begin(0x44)) {
    Serial.println("ERROR_SHT31");
    delay(20);
  }

  // Intento de Conexión: MPU6050
  while (!mpu.begin()) {
    Serial.println("ERROR_MPU6050");
    delay(20);
  }

  while (!aht.begin()) {
    Serial.println("ERROR_AHT10");
    delay(20);
  }

  // Configuración MPU6050
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
}



void loop() {
  unsigned long currentMillis = millis();

  // 1. Send CSV every "interval" seconds (non-blocking) only if enabled
  if (sendCSV && currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    getAllSensorsCSV();
    Serial.println(csvBuffer); // Send CSV over LoRa
  }

  // 2. Handle only motor commands and CSV control
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();

    if (cmd.equalsIgnoreCase(".START")) {
      if (sendCSV) {
        Serial.println("CSV_ALREADY_STARTED");
      } else {
        sendCSV = true;
        Serial.println("CSV_SENDING_STARTED");
        ledBreathing = true;
        ledFlashing = ledRainbow = ledDot = false;
      }
    }
    else if (cmd.equalsIgnoreCase(".STOP")) {
      if (!sendCSV) {
        Serial.println("CSV_ALREADY_STOPPED");
      } else {
        sendCSV = false;
        ledBreathing = false;
        Serial.println("CSV_SENDING_STOPPED");
      }
    }
    else if (cmd.startsWith(".INTERVAL")) {
      // Extract numeric part after ".INTERVAL"
      float newIntervalSec = cmd.substring(9).toFloat(); // handles decimals like 2.5
      if (newIntervalSec > 0) {
        intervalSec = newIntervalSec;
        interval = (unsigned long)(intervalSec * 1000);
        Serial.println("CSV_INTERVAL_SET_" + String(intervalSec, 2) + "sec");
      } else {
        Serial.println("ERROR_INVALID_INTERVAL");
      }
    }
    else if (cmd.startsWith(".SET")) {
      int newSteps = cmd.substring(4).toInt();
      if (newSteps > 0) {
        stepsPerRev = newSteps;
        Serial.println("ACT_STEPS_SET_" + String(stepsPerRev));
      } else {
        Serial.println("ERROR_INVALID_STEPS");
      }
    } 
    else if (cmd.equalsIgnoreCase(".W")) {
      Serial.println("ACT_FORWARD");
      currentCommand = MOTOR_BOTH_CW;
    }
    else if (cmd.equalsIgnoreCase(".S")) {
      Serial.println("ACT_BACKWARD");
      currentCommand = MOTOR_BOTH_CCW;
    }
    else if (cmd.equalsIgnoreCase(".A")) {
      Serial.println("ACT_LEFT");
      currentCommand = MOTOR_OPPOSITE_A;
    }
    else if (cmd.equalsIgnoreCase(".D")) {
      Serial.println("ACT_RIGHT");
      currentCommand = MOTOR_OPPOSITE_D;
    }

    else if (cmd.equalsIgnoreCase(".FLASH")) {
      ledFlashing = true;
      ledBreathing = ledRainbow = ledDot = ledCam = false;
      Serial.println("LED_MODE_FLASH");
    }

    else if (cmd.equalsIgnoreCase(".CAM")) {
      ledCam = true;
      ledBreathing = ledFlashing = ledRainbow = ledDot = false;
      setColor(0, 0, 50); // Blue steady
    }

    else if (cmd.equalsIgnoreCase(".RAINBOW")) {
      ledRainbow = true;
      ledBreathing = ledFlashing =  ledDot = ledCam = false;
      Serial.println("LED_MODE_RAINBOW");
    }
    // Ignorar mensajes ajenos
  }
}

String getAllSensorsCSV() {
  // Leer sensores: SHT , AHT , ESP , M1 , M2 , MPU , DIST
  float sht_temp = sht31.readTemperature();
  float sht_hum  = sht31.readHumidity();

  sensors_event_t aht_humidity, aht_temp;
  aht.getEvent(&aht_humidity, &aht_temp);

  float int_temp = aht_temp.temperature;
  float int_hum  = aht_humidity.relative_humidity;

  // V,I,P
  float espBusV = ina219_ESP.getBusVoltage_V();
  float espCurrent = ina219_ESP.getCurrent_mA();
  float espPower = ina219_ESP.getPower_mW();

  float m1BusV = ina219_M1.getBusVoltage_V();
  float m1Current = ina219_M1.getCurrent_mA();
  float m1Power = ina219_M1.getPower_mW();

  float m2BusV = ina219_M2.getBusVoltage_V();
  float m2Current = ina219_M2.getCurrent_mA();
  float m2Power = ina219_M2.getPower_mW();

  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  int dist1 = sensor1.readRangeContinuousMillimeters();
  int dist2 = sensor2.readRangeContinuousMillimeters();
  int dist3 = sensor3.readRangeContinuousMillimeters();

  // Escalar valores, 2 decimales excepto Is
  int16_t sht_temp_i = sht_temp * 100;
  int16_t sht_hum_i  = sht_hum  * 100;
  int16_t int_temp_i = int_temp * 100;
  int16_t int_hum_i  = int_hum  * 100;

  int16_t espBusV_i = espBusV * 100;
  int16_t espCurrent_i = espCurrent * 10; // 0.1mA, no overflow
  int16_t espPower_i = espPower;          // mW

  int16_t m1BusV_i = m1BusV * 100;
  int16_t m1Current_i = m1Current * 10;
  int16_t m1Power_i = m1Power;

  int16_t m2BusV_i = m2BusV * 100;
  int16_t m2Current_i = m2Current * 10;
  int16_t m2Power_i = m2Power;

  int16_t accX_i = a.acceleration.x * 100;
  int16_t accY_i = a.acceleration.y * 100;
  int16_t accZ_i = a.acceleration.z * 100;
  int16_t gyroX_i = g.gyro.x * 100;
  int16_t gyroY_i = g.gyro.y * 100;
  int16_t gyroZ_i = g.gyro.z * 100;

  // CSV
  snprintf(csvBuffer, CSV_BUFFER_SIZE,
    "%d,%d,%d,%d,"      // SHT, AHT
    "%d,%d,%d,"         // ESP
    "%d,%d,%d,"         // M1
    "%d,%d,%d,"         // M2
    "%d,%d,%d,%d,%d,%d,"// Acc + Gyro
    "%d,%d,%d",         // Dist
    sht_temp_i, sht_hum_i, int_temp_i, int_hum_i,
    espBusV_i, espCurrent_i, espPower_i,
    m1BusV_i, m1Current_i, m1Power_i,
    m2BusV_i, m2Current_i, m2Power_i,
    accX_i, accY_i, accZ_i, gyroX_i, gyroY_i, gyroZ_i,
    dist1, dist2, dist3
  );

  return String(csvBuffer);
}

void setColor(uint8_t red, uint8_t green, uint8_t blue) {
  uint32_t color = strip.Color(red, green, blue);
  for (int i = 0; i < LED_COUNT; i++) {
    strip.setPixelColor(i, color);
  }
  strip.show();
}

void motorTask(void * parameter) {
  while (true) {
    if (currentCommand != MOTOR_IDLE) {
      digitalWrite(SLEEP_PIN, HIGH);
      ledDot = true;
      ledBreathing = ledFlashing = ledRainbow = ledCam = false;

      int dir1 = LOW, dir2 = LOW;
      switch (currentCommand) {
        case MOTOR_BOTH_CW: dir1 = LOW; dir2 = LOW; break;
        case MOTOR_BOTH_CCW: dir1 = HIGH; dir2 = HIGH; break;
        case MOTOR_OPPOSITE_A: dir1 = HIGH; dir2 = LOW; break;
        case MOTOR_OPPOSITE_D: dir1 = LOW; dir2 = HIGH; break;
      }

      digitalWrite(DIR1_PIN, dir1);
      digitalWrite(DIR2_PIN, dir2);

      for (int i = 0; i < stepsPerRev; i++) {
        digitalWrite(STEP1_PIN, HIGH);
        digitalWrite(STEP2_PIN, HIGH);
        delayMicroseconds(STEP_DELAY_US);
        digitalWrite(STEP1_PIN, LOW);
        digitalWrite(STEP2_PIN, LOW);
        delayMicroseconds(STEP_DELAY_US);
      }

      digitalWrite(SLEEP_PIN, LOW);
      ledDot = false;

      if (sendCSV) {
        ledBreathing = true;
        ledFlashing = ledRainbow = ledDot = ledCam = false;
      }

      currentCommand = MOTOR_IDLE;
    }

    vTaskDelay(1);
  }
}

void ledTask(void *parameter) {
  unsigned long previousMillis = 0;
  int brightness = 0;
  int fadeAmount = 5;
  static int dotIndex = 0;
  static int dotDirection = 1;
  static uint16_t rainbowOffset = 0;

  while (true) {
    if (ledBreathing) {
      // Slow breathing purple
      float level = (sin(millis() / 500.0) + 1.0) / 2.0; // 0–1 sine wave
      setColor(0, 10 * level, 0);
      vTaskDelay(20 / portTICK_PERIOD_MS);
    }

    else if (ledFlashing) {
      // Flashing yellow
      static bool state = false;
      state = !state;
      if (state) setColor(20, 20, 0);
      else setColor(0, 0, 0);
      vTaskDelay(300 / portTICK_PERIOD_MS);
    }

    else if (ledRainbow) {
      strip.setBrightness(50);
      
      for (int i = 0; i < LED_COUNT; i++) {
        int pixelHue = (i * 65536L / LED_COUNT) + rainbowOffset;
        strip.setPixelColor(i, strip.gamma32(strip.ColorHSV(pixelHue)));
      }
      strip.show();
      rainbowOffset += 256; // controls rainbow speed
      vTaskDelay(20 / portTICK_PERIOD_MS);
    }

    else if (ledDot) {
      strip.clear();

      // Light the current LED and the next one (no wraparound)
      strip.setPixelColor(dotIndex, strip.Color(0, 0, 15));      // main blue
      strip.setPixelColor(dotIndex + 1, strip.Color(0, 0, 15));  // next LED
      strip.setPixelColor(dotIndex + 2, strip.Color(0, 0, 15));  // next LED

      strip.show();

      dotIndex += dotDirection;

      // Reverse direction when the leading LED reaches the end
      if (dotIndex >= LED_COUNT - 2 || dotIndex <= 0) {
        dotDirection = -dotDirection;
      }

      vTaskDelay(100 / portTICK_PERIOD_MS);
    }

    else if (ledCam) {
      setColor(25, 25, 25);
      strip.show();
      vTaskDelay(500 / portTICK_PERIOD_MS);
      ledCam = false;
    }

    else if (!ledBreathing && !ledFlashing && !ledRainbow && !ledDot && !ledCam) {
      float level = (sin(millis() / 500.0) + 1.0) / 2.0;
      setColor(0, 2 * level, 2 * level); // soft cyan
      vTaskDelay(30 / portTICK_PERIOD_MS);
    }

    else {
      // All off
      setColor(0, 0, 0);
      vTaskDelay(100 / portTICK_PERIOD_MS);
    }
  }
}
