#include <Wire.h>
#include <Arduino.h>
#include <Adafruit_INA219.h>
#include <Adafruit_SHT31.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_AHTX0.h>
#include <VL53L0X.h>

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
#define SLEEP1_PIN 25
#define DIR2_PIN 18
#define STEP2_PIN 19
#define SLEEP2_PIN 5
#define XSHUT1 16     //XSHUT para VL53L0X addresses
#define XSHUT2 17
#define XSHUT3 4
#define RX_PIN    3   // UART RX desde Serial entrando de LoRa
#define TX_PIN    1   // UART TX para Serial FWD LoRa

int stepsPerRev = 200;    // 1 revolución por defecto, cambiar con .STEP###
#define STEP_DELAY_US 2000   // Velocidad

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

  pinMode(DIR1_PIN, OUTPUT);
  pinMode(STEP1_PIN, OUTPUT);
  pinMode(SLEEP1_PIN, OUTPUT);
  pinMode(DIR2_PIN, OUTPUT);
  pinMode(STEP2_PIN, OUTPUT);
  pinMode(SLEEP2_PIN, OUTPUT);
  
  xTaskCreatePinnedToCore(
    motorTask,       
    "MotorTask",     
    4096,            
    NULL,            
    1,               
    NULL,            
    1                // core 1
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
  // LoRa Command handling
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();

    if (cmd.equalsIgnoreCase(".AMBTEMP")) {
      String sht_Msg;
      float sht_temp = sht31.readTemperature();
      getAmbTemp(sht_Msg);

      if (!isnan(sht_temp)) {
        Serial.println(sht_Msg);
      } else {
        Serial.println("ERR_SHT31");
      }
    }

    else if (cmd.equalsIgnoreCase(".INTTEMP")) {
      String aht_Msg;
      getIntTemp(aht_Msg);
      Serial.println(aht_Msg);
    }
      
    else if (cmd.startsWith(".POWER")) {
      int powerNum = cmd.substring(6).toInt(); // POWER1, POWER2, POWER3
      String powerMsg;

      if (powerNum == 1) {                        // ESP32
        getPower(ina219_ESP, powerNum, powerMsg);
        Serial.print(powerMsg);
      }
      else if (powerNum == 2) {                   // Motor 1
        getPower(ina219_M1, powerNum, powerMsg);
        Serial.print(powerMsg);
      }
      else if (powerNum == 3) {                   // Motor 2
        getPower(ina219_M2, powerNum, powerMsg);
        Serial.print(powerMsg);
      }
      else {
        Serial.println("ERROR_USE_.POWER1_.POWER2_.POWER3");
      }
    }

    else if (cmd.equalsIgnoreCase(".GYRO")) {
      String gyroMsg;
      getGyro(gyroMsg);
      Serial.print(gyroMsg);
    }

    else if (cmd.startsWith(".DIST")) {
      int distNum = cmd.substring(5).toInt();
      String distMsg;

      if (distNum == 1) {
        getDist(sensor1, distNum, distMsg);
        Serial.println(distMsg);
      } 
      else if (distNum == 2) {
        getDist(sensor2, distNum, distMsg);
        Serial.println(distMsg);
      } 
      else if (distNum == 3) {
        getDist(sensor3, distNum, distMsg);
        Serial.println(distMsg);
      } 
      else {
        Serial.println("ERROR_USE_.DIST1_.DIST2_.DIST3");
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

    else if (cmd.length() > 0) {
      // Si no es un comando, no hacer nada (ACK de mensaje no-comando lo hace LoRa)
    }
  }
}

void getRSSI(String &rssiMsg) {
  long rssi = WiFi.RSSI();
  rssiMsg = "TEL_WiFi_RSSI_" + String(rssi) + "_dBm";
}

void getAmbTemp(String &sht_Msg) {
  float sht_temp = sht31.readTemperature();
  float sht_hum  = sht31.readHumidity();

  sht_Msg = String("TEL_TEMP_") + String(sht_temp) + "_CELCIUS\n" +
            String("TEL_HUMI_") + String(sht_hum) + "_PERCENT";
}

void getIntTemp(String &aht_Msg) {
  sensors_event_t aht_humidity, aht_temp;
  aht.getEvent(&aht_humidity, &aht_temp);

  aht_Msg = String("TEL_TEMP_") + String(aht_temp.temperature) + "_CELCIUS\n" +
            String("TEL_HUMI_") + String(aht_humidity.relative_humidity) + "_PERCENT";
}

void getPower(Adafruit_INA219 &ina, int powerNum, String &powerMsg) {
    float busVoltage = ina.getBusVoltage_V();
    float current_mA = ina.getCurrent_mA();
    float power_mW   = ina.getPower_mW();

    powerMsg = String("TEL_VOLT")  + String(powerNum) + "_" + String(busVoltage) + "_V\n" +
               String("TEL_CURR")  + String(powerNum) + "_" + String(current_mA)  + "_mA\n" +
               String("TEL_POWER") + String(powerNum) + "_" + String(power_mW)   + "_mW";
}

void getGyro(String &gyroMsg) {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  gyroMsg = String("TEL_ACC_X_") + String(a.acceleration.x) + "_m/s^2\n" +
            String("TEL_ACC_Y_") + String(a.acceleration.y) + "_m/s^2\n" +
            String("TEL_ACC_Z_") + String(a.acceleration.z) + "_m/s^2\n" +
            String("TEL_GYR_X_") + String(g.gyro.x) + "_rad/s\n" +
            String("TEL_GYR_Y_") + String(g.gyro.y) + "_rad/s\n" +
            String("TEL_GYR_Z_") + String(g.gyro.z) + "_rad/s";
}

void getDist(VL53L0X &sensor, int distNum, String &distMsg) {
  distMsg = String("TEL_LASER") + String(distNum) + "_" +
            String(sensor.readRangeContinuousMillimeters()) + "_mm";
}

void motorTask(void * parameter) {
  while (true) {
    if (currentCommand != MOTOR_IDLE) {
      digitalWrite(SLEEP1_PIN, HIGH);
      digitalWrite(SLEEP2_PIN, HIGH);

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

      digitalWrite(SLEEP1_PIN, LOW);
      digitalWrite(SLEEP2_PIN, LOW);

      currentCommand = MOTOR_IDLE;
    }

    vTaskDelay(1);
  }
}
