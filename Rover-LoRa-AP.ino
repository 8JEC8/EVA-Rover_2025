#include <SPI.h>
#include <LoRa.h>

#define LORA_FREQ 433E6

#define LORA_SS   5    // CS
#define LORA_DIO0 25   // DIO0

void setup() {
  Serial.begin(115200);
  while (!Serial);

  SPI.begin();

  LoRa.setPins(LORA_SS, -1, LORA_DIO0);

  if (!LoRa.begin(LORA_FREQ)) {
    Serial.println("AP_ERR_LoRa_FAILED");
    while (true);
  }

  // LoRa Settings:
  LoRa.setTxPower(20);
  LoRa.setSpreadingFactor(10); // Spreading Factor
  LoRa.setSignalBandwidth(125E3); // BW
  LoRa.setCodingRate4(5); // Coding Rate
  LoRa.setSyncWord(0x88); // Sync word
  LoRa.setPreambleLength(8); // Preamble: 8 symbols
  LoRa.enableCrc(); // CRC

  Serial.println("\n");
  Serial.println("Voyager21: LoRa Communication Ready");
  Serial.println("'.COMLIST' para Lista de Comandos Disponibles");
}

void loop() {
  // RECV: LoRa
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    String received = "";
    while (LoRa.available()) {
      received += (char)LoRa.read();
    }
    String received_message = "  LORA_RECV_STA_" + received + "_" + LoRa.packetRssi() + "_dBm";
    Serial.println(received_message);
  }

  // SENT: Serial STA (WiFi + LoRa)
  if (Serial.available()) {
    String msg = Serial.readStringUntil('\n');
    msg.trim();

    if (msg.equalsIgnoreCase(".COMLIST")) {
      Serial.println(msg);
      Serial.println("  Lista de Comandos:");
      Serial.println("    '.RSSI'   : Intensidad de Señal Recibida");
      Serial.println("    '.AMBTEMP': Lectura de Temperatura y Humedad (Ambiente)");
      Serial.println("    '.INTTEMP': Lectura de Temperatura y Humedad (Interna)");
      Serial.println("    '.GYRO'   : Lectura de Acelerómetro y Giroscopio");
      Serial.println("    '.POWER#' : V,I,Pow ; # = 1 (ESP), 2 (Motor 1), 3 (Motor 3)");
      Serial.println("    '.DIST#'  : Distancia de Sensor de Laser # = 1, 2, o 3");
      Serial.println("    '.SET#'   : Selección de Pasos (200 = 1 revolución)");
      Serial.println("    '.W'      : Movimiento Hacia Adelante");
      Serial.println("    '.S'      : Movimiento Hacia Atrás");
      Serial.println("    '.A'      : Movimiento CCW");
      Serial.println("    '.D'      : Movimiento CW");
      Serial.println("    '.CALCULATE'    : Calcula la ruta");
      Serial.println("    '.AUTO'         : Realiza la ruta de manera autonoma");
      Serial.println("    '.REVERSE'      : Realiza la ruta hacia de manera inversa");
      Serial.println("    '.SHOW'         : Muestra la ruta caulculada");
      Serial.println("    '.INSTRUCTIONS' : Muestra las instrucciones para realizar la ruta");
      Serial.println("    '.CHANGE'       : Cambiar la meta actual '.CHANGEY,X'");
    } else {
      // LoRa sending
      LoRa.beginPacket();
      LoRa.print(msg);
      LoRa.endPacket();

      Serial.print("LORA_SENT_AP_");
      Serial.println(msg);
    }
  }
}
