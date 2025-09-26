#include <WiFi.h>

// Setup de WiFi (AP)
const char* ssid     = "Voyager21";     // Nombre de Red WiFi
const char* password = "Locker31";    // Contraseña

// Configuración TCP
WiFiServer server(3131);   // AP Escucha en Puerto 33  

void setup() {
  Serial.begin(115200);
  delay(500);
  // Inicializar ESP32 en Modo AP
  WiFi.softAP(ssid, password);
  Serial.println("AP Inicializado");
  Serial.print("AP IP address: ");
  Serial.println(WiFi.softAPIP());   // Desplegar IP

  // Iniciar Servidor TCP
  server.begin(3131);
  Serial.println("TCP Server Iniciado en Puerto 3131");
}

WiFiClient client;  // Declare once, outside loop()

void loop() {
  if (!client || !client.connected()) {
    client = server.available();  // Aceptar Nueva Conexión si no se ha Conectado
    if (client) {
      Serial.println("STA Conectado");
      Serial.println("'.COMLIST' para Lista de Comandos Disponibles");
    }
  }

  if (client && client.connected()) {
    // Revisar si Cliente Envió Mensaje
    if (client.available()) {
      String msg = client.readStringUntil('\n');
      msg.trim();
      if (msg.length() > 0) {
        Serial.print("  RECV_STA_");
        Serial.println(msg);
      }
    }

    // Revisar si hay Mensaje en Terminal
    if (Serial.available()) {
      String userMsg = Serial.readStringUntil('\n');
      userMsg.trim();   // remove whitespace and line endings

        if (userMsg.equalsIgnoreCase(".COMLIST")) {
          Serial.println(userMsg);
          Serial.println("  Lista de Comandos:");
          Serial.println("    '.RSSI'   : Intensidad de Señal Recibida");
          Serial.println("    '.AMBTEMP': Lectura de Temperatura y Humedad (Ambiente)");
          Serial.println("    '.INTTEMP': Lectura de Temperatura y Humedad (Interna)");
          Serial.println("    '.GYRO'   : Lectura de Acelerómetro y Giroscopio");
          Serial.println("    '.POWER#' : V,I,Pow ; # = 1 (ESP), 2 (Motor 1), 3 (Motor 3)");
          Serial.println("    '.DIST#'  : Distancia de Sensor de Laser # = 1, 2, o 3");
          Serial.println("    '.SET#'   : Selección de Pasos (200 = 1 revolución)");
          Serial.println("    '.W'      : Movimiento Hacia Adelante");
          Serial.println("    '.S'      : Movimiento Hacia Detrás");
          Serial.println("    '.A'      : Movimiento Hacia Izquierda");
          Serial.println("    '.D'      : Movimiento Hacia Derecha");
          Serial.println("    '.ROUTE_CALCULATE'    : Calcula la ruta");
          Serial.println("    '.ROUTE_AUTO'         : Realiza la ruta de manera autonoma");
          Serial.println("    '.REVERSE'            : Realiza la ruta hacia de manera inversa");
          Serial.println("    '.ROUTE_SHOW'         : Muestra la ruta caulculada");
          Serial.println("    '.ROUTE_INSTRUCTIONS' : Muestra las instrucciones para realizar la ruta");
          Serial.println("    '.ROUTE_CHANGE'       : Cambiar la meta actual");
        } else {
            client.println(userMsg);       // Enviado a STA
            Serial.print("SENT_AP_");      // Formato de Mensajes Normales
            Serial.println(userMsg);
        }
      }
  }
  delay(50);
}
