#include <WiFi.h>

// WiFi
const char* ssid = "Voyager21_AP";
const char* password = "12345678";

bool staEnabled = false; 
wl_status_t lastStatus = WL_NO_SHIELD;  // Track Status

// WiFi
WiFiClient client;    
const uint16_t AP_PORT = 3131; 
const char* AP_IP = "192.168.4.1";

// Use UART2 for communication with LoRa ESP
HardwareSerial LoRaSerial(2); // UART2

#define UART_RX 19
#define UART_TX 20

void startSTA() {
  if (!staEnabled) {
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
    delay(100);

    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);
    LoRaSerial.println("STA_CONN_ATTEMPT");

    unsigned long startAttempt = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - startAttempt < 5000) {
      delay(500);
    }

    if (WiFi.status() == WL_CONNECTED) {
      LoRaSerial.println("STA_ENABLED");
      LoRaSerial.print("CONNECTED_");
      LoRaSerial.println(WiFi.localIP());

      // Try TCP connect
      if (client.connect(AP_IP, AP_PORT)) {
        LoRaSerial.println("STA_TCP_CONNECTED");
      } else {
        LoRaSerial.println("STA_TCP_ERR_CONNECT");
      }

      staEnabled = true;
      lastStatus = WL_CONNECTED;
    } else {
      LoRaSerial.println("STA_ERR_CONNECTION");
      WiFi.disconnect(true);
      WiFi.mode(WIFI_OFF);
      staEnabled = false;
      lastStatus = WL_NO_SHIELD;
    }
  } else {
    LoRaSerial.println("STA_WiFi_RUNNING");
  }
}

void stopSTA() {
  if (staEnabled) {
    WiFi.disconnect(true);  // disconnect and erase config
    WiFi.mode(WIFI_OFF);    // turn WiFi radio fully off
    LoRaSerial.println("STA_DISABLED");
    staEnabled = false;
    lastStatus = WL_NO_SHIELD;
  } else {
    LoRaSerial.println("STA_WiFi_NOTRUNNING");
  }
}

void setup() {
  // Initialize UART2
  LoRaSerial.begin(115200, SERIAL_8N1, UART_RX, UART_TX);
  delay(100); // allow serial to settle
}

void loop() {
  // LoRa Command handling
  if (LoRaSerial.available()) {
    String cmd = LoRaSerial.readStringUntil('\n');
    cmd.trim();

    if (cmd.equalsIgnoreCase(".STAON")) {
      startSTA();
    } else if (cmd.equalsIgnoreCase(".STAOFF")) {
      stopSTA();
    } else if (cmd.length() > 0) {
      // If not a command, dont do anything;
    }
  }

  // WiFi handling
  if (staEnabled && client && client.connected() && client.available()) {
    String wifiCmd = client.readStringUntil('\n');
    wifiCmd.trim();

    if (wifiCmd.length() > 0) {
      // WiFi ACK
      client.print("ACK_");
      client.println(wifiCmd);
    }
  }

  // Connection States
  wl_status_t currentStatus = WiFi.status();

  if (currentStatus != lastStatus) {
    if (lastStatus == WL_CONNECTED && currentStatus != WL_CONNECTED) {
      LoRaSerial.println("STA_DISCONNECTED");
      stopSTA();  // Fully shut down WiFi when AP disappears
    } else if (lastStatus != WL_CONNECTED && currentStatus == WL_CONNECTED) {
      LoRaSerial.println("STA_RECONNECTED");  // Optional, but won’t trigger since STA is off
      LoRaSerial.print("CONNECTED_");
      LoRaSerial.println(WiFi.localIP());
      staEnabled = true;
    }
    lastStatus = currentStatus;
  }
}
