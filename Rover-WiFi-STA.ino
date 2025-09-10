#include <WiFi.h>
#include <Wire.h>
#include <Adafruit_INA219.h>
#include <Adafruit_SHT31.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_AHTX0.h>
#include <VL53L0X.h>
#include <Adafruit_NeoPixel.h>
#define LED_COUNT 8
#define LED_PIN1 1
#define LED_PIN2 3

VL53L0X sensor1;
VL53L0X sensor2;
VL53L0X sensor3;

Adafruit_NeoPixel strip1(LED_COUNT, LED_PIN1, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel strip2(LED_COUNT, LED_PIN2, NEO_GRB + NEO_KHZ800);

Adafruit_INA219 ina219_ESP(0x40);
Adafruit_INA219 ina219_M1(0x41);
Adafruit_INA219 ina219_M2(0x45);
Adafruit_SHT31 sht31 = Adafruit_SHT31();
Adafruit_MPU6050 mpu;
Adafruit_AHTX0 aht;

// Setup de WiFi (AP)
const char* ssid     = "Voyager21";  // ID:         AP
const char* password = "Locker31";   // Contraseña: AP

// // Configuración TCP
const char* host = "192.168.4.1";   // IP de ESP32
const uint16_t port = 3131;         // Puerto de AP

WiFiClient client;

#define DIR1_PIN 27
#define STEP1_PIN 26
#define SLEEP1_PIN 25
#define DIR2_PIN 18
#define STEP2_PIN 19
#define SLEEP2_PIN 5

#define XSHUT1 16
#define XSHUT2 17
#define XSHUT3 4

int stepsPerRev = 200;       // 1 revolución por defecto, cambiar con .STEP###
#define STEP_DELAY_US 2000   // Velocidad

void setup() {
  Serial.begin(115200);
  
  strip1.begin();           // Iniciar Strip
  strip1.show();            // Apagar LEDs
  strip1.setBrightness(10); // Brillo (0-255)
  strip2.begin();
  strip2.show();
  strip2.setBrightness(10);

  delay(20);
  Serial.print("Conectando a AP: ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  
  // Esperar hasta que la conexión WiFi se establezca
  while (WiFi.status() != WL_CONNECTED) {
    delay(3000);
    Serial.print(".");
  }

  // Conexión establecida
  Serial.println("\nConectado a AP");
  Serial.print("STA IP: ");
  Serial.println(WiFi.localIP().toString());  // keep toString()

  // Intentar Conectar a Servidor TCP en AP
  Serial.print("Conectando al servidor TCP en ");
  Serial.print(host);
  Serial.print(":");
  Serial.println(port);

  if (client.connect(host, port)) {
    client.println("SOCKET_SUCCESSFUL_CONNECTION");
  } else {
    client.println("ERROR_CONNECTION_AP");
  }

  pinMode(DIR1_PIN, OUTPUT);
  pinMode(STEP1_PIN, OUTPUT);
  pinMode(SLEEP1_PIN, OUTPUT);
  pinMode(DIR2_PIN, OUTPUT);
  pinMode(STEP2_PIN, OUTPUT);
  pinMode(SLEEP2_PIN, OUTPUT);
  
  pinMode(XSHUT1, OUTPUT);
  pinMode(XSHUT2, OUTPUT);
  pinMode(XSHUT3, OUTPUT);

  Wire.begin();
  Serial.setDebugOutput(false);

  // Apagamos todos los sensores
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
    Serial.println("No se pudo encontrar INA219 del ESP32, reintentando...");
    delay(20);
  }
  Serial.println("INA219 Listo");

  while (!ina219_M1.begin()) {
    Serial.println("No se pudo encontrar INA219 del Motor 1, reintentando...");
    delay(20);
  }
  Serial.println("INA219 Listo");

  while (!ina219_M2.begin()) {
    Serial.println("No se pudo encontrar INA219 del Motor 12, reintentando...");
    delay(20);
  }
  Serial.println("INA219 Listo");

  // Intento de Conexión: SHT31
  while (!sht31.begin(0x44)) {  // default I2C addr 0x44
    Serial.println("No se pudo encontrar SHT31, reintentando...");
    delay(20);
  }
  Serial.println("SHT31 Listo.");

  // Intento de Conexión: MPU6050
  while (!mpu.begin()) {
    Serial.println("No se pudo encontrar MPU6050, reintentando...");
    delay(20);
  }
  Serial.println("MPU6050 Listo.");

  while (!aht.begin()) {
    Serial.println("No se pudo encontrar AHT10, reintentando...");
    delay(20);
  }
  Serial.println("AHT10 Listo.");

  // Configuración MPU6050
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
}

void loop() {
  // Si Existe Conexión, Permitir Comunicación
  if (client.connected()) {
    // Revisar Mensajes Entrantes de AP
    if (client.available()) {
      String msg = client.readStringUntil('\n');
      Serial.print("  RECV_AP_");
      Serial.println(msg);

      String ack = "ACK_" + String(msg);
      client.println(ack);

      msg.trim();

      if (msg.equalsIgnoreCase(".RSSI")) {
        long rssi = WiFi.RSSI();
        String rssiMsg = "TEL_RSSI_" + String(rssi) + "_dBm";
        client.println(rssiMsg);
        rainbowCycle(10);
      }

      else if (msg.equalsIgnoreCase(".AMBTEMP")) {
        float sht_temp = sht31.readTemperature();
        float sht_hum  = sht31.readHumidity();

        if (!isnan(sht_temp)) {
          String sht_tempMsg = "TEL_TEMP_" + String(sht_temp) + "_CELCIUS";
          String sht_humMsg  = "TEL_HUMI_"  + String(sht_hum)  + "_PERCENT";
          client.println(sht_tempMsg);
          client.println(sht_humMsg);
        } else {
          client.println("ERR_SHT31");
        }
      }

      else if (msg.equalsIgnoreCase(".INTTEMP")) {
        sensors_event_t aht_humidity, aht_temp;
        aht.getEvent(&aht_humidity, &aht_temp);

        String aht_tempMsg = "TEL_TEMP_" + String(aht_temp.temperature) + "_CELCIUS";
        String aht_humMsg  = "TEL_HUMI_" + String(aht_humidity.relative_humidity) + "_PERCENT";

        client.println(aht_tempMsg);
        client.println(aht_humMsg);
      }
      
      else if (msg.equalsIgnoreCase(".INTTEMP")) {
        sensors_event_t aht_humidity, aht_temp;
        aht.getEvent(&aht_humidity, &aht_temp);

        String aht_tempMsg = "TEL_TEMP_" + String(aht_temp.temperature) + "_CELCIUS";
        String aht_humMsg  = "TEL_HUMI_" + String(aht_humidity.relative_humidity) + "_PERCENT";

        client.println(aht_tempMsg);
        client.println(aht_humMsg);
      }


      else if (msg.startsWith(".POWER")) {
        int powerNum = msg.substring(6).toInt(); // 1,2,3 luego de .POWER

        if (powerNum == 1) {
          float busVoltage = ina219_ESP.getBusVoltage_V();
          float current_mA = ina219_ESP.getCurrent_mA();
          float power_mW   = ina219_ESP.getPower_mW();

          String TEL_V = "TEL_VOLT1_" + String(busVoltage) + "_V";
          String TEL_I = "TEL_CURR1_" + String(current_mA) + "_mA";
          String TEL_P = "TEL_POWER1_" + String(power_mW) + "_mW";
          client.println(TEL_V);
          client.println(TEL_I);
          client.println(TEL_P);
        } 
        else if (powerNum == 2) {
          float busVoltage = ina219_M1.getBusVoltage_V();
          float current_mA = ina219_M1.getCurrent_mA();
          float power_mW   = ina219_M1.getPower_mW();

          String TEL_V = "TEL_VOLT2_" + String(busVoltage) + "_V";
          String TEL_I = "TEL_CURR2_" + String(current_mA) + "_mA";
          String TEL_P = "TEL_POWER2_" + String(power_mW) + "_mW";
          client.println(TEL_V);
          client.println(TEL_I);
          client.println(TEL_P);
        } 
        else if (powerNum == 3) {
          float busVoltage = ina219_M2.getBusVoltage_V();
          float current_mA = ina219_M2.getCurrent_mA();
          float power_mW   = ina219_M2.getPower_mW();

          String TEL_V = "TEL_VOLT3_" + String(busVoltage) + "_V";
          String TEL_I = "TEL_CURR3_" + String(current_mA) + "_mA";
          String TEL_P = "TEL_POWER3_" + String(power_mW) + "_mW";
          client.println(TEL_V);
          client.println(TEL_I);
          client.println(TEL_P);
        } 
        else {
          String TEL_ERROR_INA = "ERROR_USE_.POWER1_.POWER2_.POWER3";
          client.println(TEL_ERROR_INA);
        }
      }

      else if (msg.equalsIgnoreCase(".GYRO")) {
        sensors_event_t a, g, temp;
        mpu.getEvent(&a, &g, &temp);

        String ACCX = "TEL_ACC_X_" + String(a.acceleration.x) + "_m/s^2";
        String ACCY = "TEL_ACC_Y_" + String(a.acceleration.y) + "_m/s^2";
        String ACCZ = "TEL_ACC_Z_" + String(a.acceleration.z) + "_m/s^2";
        String GYRX = "TEL_GYR_X_" + String(g.gyro.x) + "_rad/s";
        String GYRY = "TEL_GYR_Y_" + String(g.gyro.y) + "_rad/s";
        String GYRZ = "TEL_GYR_Z_" + String(g.gyro.z) + "_rad/s";
        client.println(ACCX);
        client.println(ACCY);
        client.println(ACCZ);
        client.println(GYRX);
        client.println(GYRY);
        client.println(GYRZ);
      }

      else if (msg.startsWith(".DIST")) {
        int distNum = msg.substring(5).toInt();
        String TEL_DIST;

        if (distNum == 1) {
          TEL_DIST = "TEL_LASER1_" + String(sensor1.readRangeContinuousMillimeters()) + "_mm";
        } else if (distNum == 2) {
          TEL_DIST = "TEL_LASER2_" + String(sensor2.readRangeContinuousMillimeters()) + "_mm";
        } else if (distNum == 3) {
          TEL_DIST = "TEL_LASER3_" + String(sensor3.readRangeContinuousMillimeters()) + "_mm";
        } else {
          TEL_DIST = "ERROR_";
        }

        client.println(TEL_DIST);
      }

      else if (msg.startsWith(".SET")) {
        int newSteps = msg.substring(4).toInt();
        if (newSteps > 0) {
          stepsPerRev = newSteps;
          client.println("ACT_STEPS_SET_" + String(stepsPerRev));
        } else {
          client.println("ERROR_INVALID_STEPS");
        }
      } 

      else if (msg.equalsIgnoreCase(".W")) {
        client.println("ACT_FORWARD");
        moveBothCW();
      }

      else if (msg.equalsIgnoreCase(".S")) {
        client.println("ACT_BACKWARD");
        moveBothCCW();
      }

      else if (msg.equalsIgnoreCase(".A")) {
        client.println("ACT_LEFT");
        moveOppositeA();
      }

      else if (msg.equalsIgnoreCase(".D")) {
        client.println("ACT_RIGHT");
        moveOppositeD();
      }
    }

  } else {
    // Si se desconecta, intentar reconectar
    Serial.println("SOCKET_DISCONNECTED_RETRYING");
    if (client.connect(host, port)) {
      Serial.println("SOCKET_RECONNECTED");
    }
    delay(5000);
  }

  delay(50);
}

void moveBothCCW() {
  digitalWrite(SLEEP1_PIN, HIGH);
  digitalWrite(SLEEP2_PIN, HIGH);

  digitalWrite(DIR1_PIN, HIGH);
  digitalWrite(DIR2_PIN, HIGH);
  stepBoth(stepsPerRev);

  digitalWrite(SLEEP1_PIN, LOW);
  digitalWrite(SLEEP2_PIN, LOW);
}

void moveBothCW() {
  digitalWrite(SLEEP1_PIN, HIGH);
  digitalWrite(SLEEP2_PIN, HIGH);

  digitalWrite(DIR1_PIN, LOW);
  digitalWrite(DIR2_PIN, LOW);
  stepBoth(stepsPerRev);

  digitalWrite(SLEEP1_PIN, LOW);
  digitalWrite(SLEEP2_PIN, LOW);
}

void moveOppositeA() {
  digitalWrite(SLEEP1_PIN, HIGH);
  digitalWrite(SLEEP2_PIN, HIGH);

  digitalWrite(DIR1_PIN, HIGH);
  digitalWrite(DIR2_PIN, LOW);
  stepBoth(stepsPerRev);

  digitalWrite(SLEEP1_PIN, LOW);
  digitalWrite(SLEEP2_PIN, LOW);
}

void moveOppositeD() {
  digitalWrite(SLEEP1_PIN, HIGH);
  digitalWrite(SLEEP2_PIN, HIGH);

  digitalWrite(DIR1_PIN, LOW);
  digitalWrite(DIR2_PIN, HIGH);
  stepBoth(stepsPerRev);

  digitalWrite(SLEEP1_PIN, LOW);
  digitalWrite(SLEEP2_PIN, LOW);
}

void stepBoth(int steps) {
  for (int i = 0; i < steps; i++) {
    digitalWrite(STEP1_PIN, HIGH);
    digitalWrite(STEP2_PIN, HIGH);
    delayMicroseconds(STEP_DELAY_US);
    digitalWrite(STEP1_PIN, LOW);
    digitalWrite(STEP2_PIN, LOW);
    delayMicroseconds(STEP_DELAY_US);
  }
}

void rainbowCycle(int wait) {
  uint16_t i, j;
  for (j = 0; j < 256 * 5; j++) { // 5 cycles of all colors
    for (i = 0; i < LED_COUNT; i++) {
      uint32_t color = Wheel(((i * 256 / LED_COUNT) + j) & 255);
      strip1.setPixelColor(i, color);
      strip2.setPixelColor(i, color);
    }
    strip1.show();
    strip2.show();
    delay(wait);
  }
}

uint32_t Wheel(byte WheelPos) {
  WheelPos = 255 - WheelPos;
  if (WheelPos < 85) {
    return strip1.Color(255 - WheelPos * 3, 0, WheelPos * 3);
  }
  if (WheelPos < 170) {
    WheelPos -= 85;
    return strip1.Color(0, WheelPos * 3, 255 - WheelPos * 3);
  }
  WheelPos -= 170;
  return strip1.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
}