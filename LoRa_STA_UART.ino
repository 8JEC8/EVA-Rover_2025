#include <WiFi.h>

// WiFi
const char* ssid = "Voyager21_AP";
const char* password = "12345678";

bool staEnabled = false; 
wl_status_t lastStatus = WL_NO_SHIELD;  // Track last known status

void startSTA() {
  if (!staEnabled) {
    // --- Full reset of WiFi stack ---
    WiFi.disconnect(true);  // erase previous config
    WiFi.mode(WIFI_OFF);    // turn off WiFi
    delay(100);             // allow driver to settle

    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);
    Serial.print("STA_CONN_ATTEMPT");

    unsigned long startAttempt = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - startAttempt < 5000) {
      delay(500);
    }

    if (WiFi.status() == WL_CONNECTED) {
      Serial.println("\nSTA_ENABLED");
      Serial.print("CONNECTED_");
      Serial.println(WiFi.localIP());
      staEnabled = true;
      lastStatus = WL_CONNECTED;
    } else {
      Serial.println("\nSTA_ERR_CONNECTION");
      WiFi.disconnect(true);
      WiFi.mode(WIFI_OFF);
      staEnabled = false;
      lastStatus = WL_NO_SHIELD;
    }
  } else {
    Serial.println("STA_WiFi_RUNNING");
  }
}

void stopSTA() {
  if (staEnabled) {
    WiFi.disconnect(true);  // disconnect and erase config
    WiFi.mode(WIFI_OFF);    // turn WiFi radio fully off
    Serial.println("STA_DISABLED");
    staEnabled = false;
    lastStatus = WL_NO_SHIELD;
  } else {
    Serial.println("STA_WiFi_NOTRUNNING");
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial);

}

void loop() {
  // --- Command handling ---
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();

    if (cmd.equalsIgnoreCase(".STAON")) {
      startSTA();
    } else if (cmd.equalsIgnoreCase(".STAOFF")) {
      stopSTA();
    } else if (cmd.length() > 0) {
      Serial.println(cmd);
    }
  }

  // --- Connection monitoring ---
  wl_status_t currentStatus = WiFi.status();

  if (currentStatus != lastStatus) {
    if (lastStatus == WL_CONNECTED && currentStatus != WL_CONNECTED) {
      Serial.println("STA_DISCONNECTED");
      staEnabled = false;       // automatically update flag
    } else if (lastStatus != WL_CONNECTED && currentStatus == WL_CONNECTED) {
      Serial.println("STA_RECONNECTED");
      Serial.print("CONNECTED_");
      Serial.println(WiFi.localIP());
      staEnabled = true;        // automatically update flag
    }
    lastStatus = currentStatus;
  }
}