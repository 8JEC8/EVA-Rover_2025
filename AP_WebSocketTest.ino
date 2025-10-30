#include <WiFi.h>
#include <WebSocketsServer.h>
#include <WebServer.h> 
#include <SPI.h>
#include <LoRa.h>

// ====== Embedded HTML======
const char index_html[] PROGMEM = R"rawliteral(

<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>ESP32 Telemetry Dashboard</title>
<style>
  body {
    font-family: Arial, sans-serif;
    background: #f0f0f0;
    margin: 0;
    padding: 20px;
  }

  h2 {
    text-align: center;
    margin-bottom: 20px;
  }

  /* Main container: 3 columns */
  .telemetry-container {
    display: flex;
    gap: 20px;
    align-items: flex-start;
    justify-content: flex-start;
    flex-wrap: wrap;
  }

  .column {
    display: flex;
    flex-direction: column;
    gap: 15px;
  }

  .left-column { flex: 0 1 320px; }
  .middle-column { flex: 0 1 320px; }
  .right-column { flex: 1 1 600px; }

  .telemetry-box {
    background: #fff;
    border-radius: 12px;
    padding: 15px 20px;
    box-shadow: 0 2px 6px rgba(0,0,0,0.15);
    text-align: left;
  }

  .telemetry-box h3 {
    text-align: center;
    margin-top: 0;
    margin-bottom: 10px;
    color: #333;
  }

  .telemetry-grid {
    display: grid;
    grid-template-columns: 1fr auto;
    gap: 6px 8px;
    font-size: 1rem;
    align-items: center;
  }

  .telemetry-grid div:nth-child(odd) { text-align: left; color: #333; }
  .telemetry-grid div:nth-child(even) { text-align: right; font-weight: 600; color: #222; white-space: nowrap; }
  .telemetry-grid div:nth-child(odd):not(:last-of-type) { border-bottom: 1px solid #eee; }

  .camera-box {
    background: #fff;
    border-radius: 12px;
    padding: 15px 20px;
    box-shadow: 0 2px 6px rgba(0,0,0,0.15);
    text-align: center;
    height: 442px;
    display: flex;
    flex-direction: column;
    justify-content: center;
  }

  .camera-box img {
    width: 100%;
    height: 100%;
    object-fit: contain;
    border-radius: 8px;
  }

  /* Controls section */
  .controls-line {
    margin: 20px 0;
    border-top: 4px solid #000;
  }

  .controls-container {
    display: flex;
    gap: 20px;
    justify-content: center;
    margin-top: 20px;
    flex-wrap: wrap;
  }

  .controls-box {
    background: #fff;
    border-radius: 12px;
    padding: 10px;
    display: grid;
    gap: 1px;
    justify-items: center;
    align-items: center;
    box-shadow: 0 2px 6px rgba(0,0,0,0.15);
  }

  /* Arrow buttons grid */
  .arrow-buttons {
    grid-template-areas:
      ". up ."
      "left . right"
      ". down .";
    width: 400px;
    height: 400px;
  }

  .control-button {
    padding: 20px;
    font-size: 4rem;
    border: none;
    border-radius: 8px;
    background-color: #ff5e00;
    color: white;
    cursor: pointer;
    width: 120px;
    height: 120px;
  }

  .control-button:hover {
  background-color: #db5000;
  transform: scale(1.05);
  }

  .command-button {
  padding: 20px;
  font-size: 1.5rem;
  border: none;
  border-radius: 15px;
  background-color: #0077cc; /* Blue tone */
  color: white;
  cursor: pointer;
  width: 170px;
  height: 170px;
  transition: background-color 0.2s ease, transform 0.1s ease;
  }

  .command-button:hover {
  background-color: #005fa3;
  transform: scale(1.05);
  }

  .control-up { grid-area: up; }
  .control-down { grid-area: down; }
  .control-left { grid-area: left; }
  .control-right { grid-area: right; }

  .command-button-box {
  display: flex;
  justify-content: center;
  align-items: center;
  gap: 0px 30px;              /* spacing between the two buttons */
  width: 400px;
  height: 400px;
  flex-wrap: wrap;        /* ensures wrapping on small screens */
  }

  /* Responsive */
  @media (max-width: 900px) {
    .telemetry-container {
      flex-direction: column;
      align-items: center;
    }
    .left-column, .middle-column, .right-column {
      flex: 1 1 100%;
      max-width: none;
    }
  }

  @media (max-width: 600px) {
    .controls-container {
      flex-direction: column;
      align-items: center;
    }
    .arrow-buttons, .image-button-box {
      width: 90%;
      height: auto;
    }
    .control-button {
      width: 80px;
      height: 80px;
      font-size: 2.5rem;
    }
  }
</style>
</head>
<body>
<h2>EVA: Dashboard de Telemetría y Control</h2>

<div class="telemetry-container">
  <!-- Left Column -->
  <div class="column left-column">
    <div class="telemetry-box">
      <h3>Intensidad de Señal</h3>
      <div class="telemetry-grid">
        <div>RSSI Actual:</div><div id="rssiCurr">-- [dBm]</div>
        <div>RSSI Promediado:</div><div id="rssiAvg">-- [dBm]</div>
      </div>
    </div>

    <div class="telemetry-box">
      <h3>Sensores de Distancia</h3>
      <div class="telemetry-grid">
        <div>Distancia 1:</div><div id="dist1">-- [mm]</div>
        <div>Distancia 2:</div><div id="dist2">-- [mm]</div>
        <div>Distancia 3:</div><div id="dist3">-- [mm]</div>
      </div>
    </div>

    <div class="telemetry-box">
      <h3>Accel & Gyro</h3>
      <div class="telemetry-grid">
        <div>Accel X:</div><div id="accX">-- [g]</div>
        <div>Accel Y:</div><div id="accY">-- [g]</div>
        <div>Accel Z:</div><div id="accZ">-- [g]</div>
        <div>ΔÁngulo X:</div><div id="angX">-- [°/s]</div>
        <div>ΔÁngulo Y:</div><div id="angY">-- [°/s]</div>
        <div>ΔÁngulo Z:</div><div id="angZ">-- [°/s]</div>
      </div>
    </div>
  </div>

  <!-- Middle Column -->
  <div class="column middle-column">
    <div class="telemetry-box">
      <h3>Temperatura y Humedad</h3>
      <div class="telemetry-grid">
        <div>Temp. Interna:</div><div id="tempInt">-- [°C]</div>
        <div>Hum. Interna:</div><div id="humInt">-- [%]</div>
        <div>Temp. Externa:</div><div id="tempExt">-- [°C]</div>
        <div>Hum. Externa:</div><div id="humExt">-- [%]</div>
      </div>
    </div>

    <div class="telemetry-box">
      <h3>Lecturas de Alimentación</h3>
      <div class="telemetry-grid">
        <div>ESP+CAM Voltaje:</div><div id="voltEsp">-- [V]</div>
        <div>ESP+CAM Corriente:</div><div id="currEsp">-- [mA]</div>
        <div>ESP+CAM Potencia:</div><div id="powEsp">-- [mW]</div>
        <div>Motor 1 Voltaje:</div><div id="voltM1">-- [V]</div>
        <div>Motor 1 Corriente:</div><div id="currM1">-- [mA]</div>
        <div>Motor 1 Potencia:</div><div id="powM1">-- [mW]</div>
        <div>Motor 2 Voltaje:</div><div id="voltM2">-- [V]</div>
        <div>Motor 2 Corriente:</div><div id="currM2">-- [mA]</div>
        <div>Motor 2 Potencia:</div><div id="powM2">-- [mW]</div>
      </div>
    </div>
  </div>

  <!-- Right Column -->
  <div class="column right-column">
    <div class="camera-box">
      <h3>Última Imagen Guardada</h3>
      <img id="camImage" src="" alt="ESPCAM">
    </div>
  </div>
</div>

<!-- Line separator -->
<div class="controls-line"></div>

<!-- Controls -->
<div class="controls-container">
  <div class="controls-box arrow-buttons">
    <button id="btnUp" class="control-button control-up">↑</button>
    <button id="btnLeft" class="control-button control-left">←</button>
    <button id="btnRight" class="control-button control-right">→</button>
    <button id="btnDown" class="control-button control-down">↓</button>
  </div>  

  <div class="controls-box command-button-box">
    <button id="requestImage" class="command-button" style="background-color: #28a745;">Obtener Imagen</button>
    <button id="captureImage" class="command-button" style="background-color: #28a745;">Solicitar Imagen</button>
    <button id="recvTel" class="command-button" style="background-color: #28a7a5;">Recibir Telemetría</button>
    <button id="stopTel" class="command-button" style="background-color: #28a7a5;">Detener Telemetría</button>
  </div>

  <div class="controls-box command-button-box">
    <button id="forceShort" class="command-button">LoRa: <br> <br> Corto Alcance</button>
    <button id="forceMid" class="command-button">LoRa: <br> <br> Medio Alcance</button>
    <button id="forceLong" class="command-button">LoRa: <br> <br> Largo Alcance</button>
  </div>

</div>


<script>
  // Example: set Base64 image
  const base64String = "";
  document.getElementById('camImage').src = "data:image/jpeg;base64," + base64String;
</script>
<script src="main.js"></script>
</body>
</html>

)rawliteral";

// ====== Embedded JS======
const char main_js[] PROGMEM = R"rawliteral(

document.addEventListener('DOMContentLoaded', () => {
  const ws = new WebSocket('ws://192.168.4.1:81');

  ws.onopen = () => {
    console.log("WebSocket connected");
  };

  ws.onmessage = (event) => {
    const msg = event.data;
    console.log("Message received:", msg);

    if (msg.startsWith("IMG_")) {
      console.log("Image data received");
      const b64 = msg.substring(4);
      document.getElementById('camImage').src = "data:image/jpeg;base64," + b64;
      console.log("Image updated in HTML");
    } else {
      const data = msg.split(',');
      console.log("Parsed data:", data);

      if (data.length === 24) {
        console.log("Valid telemetry packet (24 values)");

        // --- Update telemetry values ---
        document.getElementById('rssiCurr').textContent = data[0];
        document.getElementById('rssiAvg').textContent = data[1];
        document.getElementById('tempInt').textContent = (data[2] / 100).toFixed(2);
        document.getElementById('humInt').textContent = (data[3] / 100).toFixed(2);
        document.getElementById('tempExt').textContent = (data[4] / 100).toFixed(2);
        document.getElementById('humExt').textContent = (data[5] / 100).toFixed(2);

        document.getElementById('voltEsp').textContent = (data[6] / 100).toFixed(2);
        document.getElementById('currEsp').textContent = (data[7] / 10).toFixed(1);
        document.getElementById('powEsp').textContent = data[8];

        document.getElementById('voltM1').textContent = (data[9] / 100).toFixed(2);
        document.getElementById('currM1').textContent = (data[10] / 10).toFixed(2);
        document.getElementById('powM1').textContent = data[11];

        document.getElementById('voltM2').textContent = (data[12] / 100).toFixed(2);
        document.getElementById('currM2').textContent = (data[13] / 10).toFixed(1);
        document.getElementById('powM2').textContent = data[14];

        document.getElementById('accX').textContent = (data[15] / 100).toFixed(2);
        document.getElementById('accY').textContent = (data[16] / 100).toFixed(2);
        document.getElementById('accZ').textContent = (data[17] / 100).toFixed(2);

        document.getElementById('angX').textContent = (data[18] / 100).toFixed(2);
        document.getElementById('angY').textContent = (data[19] / 100).toFixed(2);
        document.getElementById('angZ').textContent = (data[20] / 100).toFixed(2);

        document.getElementById('dist1').textContent = data[21];
        document.getElementById('dist2').textContent = data[22];
        document.getElementById('dist3').textContent = data[23];

        // --- Verify DOM updates ---
        console.log("DOM values check:", {
          rssiCurr: document.getElementById('rssiCurr').textContent,
          tempInt: document.getElementById('tempInt').textContent,
          voltEsp: document.getElementById('voltEsp').textContent,
          dist1: document.getElementById('dist1').textContent
        });
      } else {
        console.warn("Invalid telemetry length:", data.length, "values received");
      }
    }
  };

  function sendCommand(cmd) {
    if (ws.readyState === WebSocket.OPEN) {
      ws.send(cmd);
      console.log("Command sent:", cmd);
    } else {
      console.warn("WebSocket not open, cannot send:", cmd);
    }
  }

  // Arrow buttons
  document.getElementById('btnUp').addEventListener('click', () => sendCommand('UP'));
  document.getElementById('btnDown').addEventListener('click', () => sendCommand('DOWN'));
  document.getElementById('btnLeft').addEventListener('click', () => sendCommand('LEFT'));
  document.getElementById('btnRight').addEventListener('click', () => sendCommand('RIGHT'));

  // Image & telemetry buttons
  document.getElementById('requestImage').addEventListener('click', () => sendCommand('WEB_IMG'));
  document.getElementById('captureImage').addEventListener('click', () => sendCommand('CAPTURE_IMG'));
  document.getElementById('recvTel').addEventListener('click', () => sendCommand('START_TEL'));
  document.getElementById('stopTel').addEventListener('click', () => sendCommand('STOP_TEL'));

  // LoRa buttons
  document.getElementById('forceShort').addEventListener('click', () => sendCommand('LORA_SHORT'));
  document.getElementById('forceMid').addEventListener('click', () => sendCommand('LORA_MEDIUM'));
  document.getElementById('forceLong').addEventListener('click', () => sendCommand('LORA_LONG'));
});

)rawliteral";

// ====== SoftAP settings ======
const char* apSSID = "EVA_Dashboard";
const char* apPassword = "12345678";  // min 8 chars

// ====== WebSocket server ======
WebServer server(80);
WebSocketsServer webSocket = WebSocketsServer(81);

// Only first client can control
uint8_t controllerClient = 255; // 255 = no controller yet

// ====== Telemetry ======
String telemetryCSV = "";  // Latest telemetry CSV

// ====== Base64 image ======
String base64Image = "iVBORw0KGgoAAAANSUhEUgAAAAUAAAAFCAYAAACNbyblAAAAHElEQVQI12P4//8/w38GIAXDIBKE0DHxgljNBAAO9TXL0Y4OHwAAAABJRU5ErkJggg==";

// ====== LoRa pins ======
#define LORA_FREQ 433E6
#define LORA_SS   5    // CS
#define LORA_DIO0 25   // DIO0

void setup() {
  Serial.begin(115200);

  // ====== Start SoftAP ======
  WiFi.softAP(apSSID, apPassword);
  IPAddress IP = WiFi.softAPIP();
  Serial.println("SoftAP started!");
  Serial.print("Connect to ESP32 at IP: ");
  Serial.println(IP);

  // ====== Serve HTML page ======
  server.on("/", HTTP_GET, []() {
    server.send_P(200, "text/html", index_html);
  });

  // ====== Serve JS file ======
  server.on("/main.js", HTTP_GET, []() {
    server.send_P(200, "application/javascript", main_js);
  });

  server.begin();
  Serial.println("HTTP server started!");

  // ====== WebSocket server ======
  webSocket.begin();
  webSocket.onEvent(onWebSocketEvent);
  Serial.println("WebSocket server started!");

  // ====== LoRa setup ======
  SPI.begin();
  LoRa.setPins(LORA_SS, -1, LORA_DIO0);
  if (!LoRa.begin(LORA_FREQ)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }

  LoRa.setTxPower(20);
  LoRa.setSpreadingFactor(7);
  LoRa.setSignalBandwidth(250E3);
  LoRa.setCodingRate4(5);
  LoRa.setSyncWord(0x88);
  LoRa.setPreambleLength(6);
  LoRa.enableCrc();

  Serial.println("LoRa initialized!");
}

void loop() {
  webSocket.loop();
  receiveLoRaTelemetry();
  server.handleClient();
}

// ====== WebSocket events ======
void onWebSocketEvent(uint8_t client_num, WStype_t type, uint8_t * payload, size_t length) {
  switch(type) {
    case WStype_CONNECTED: {
      Serial.printf("Client %u connected\n", client_num);
      // Assign first controller if none
      if(controllerClient == 255) controllerClient = client_num;
      break;
    }
    case WStype_DISCONNECTED: {
      Serial.printf("Client %u disconnected\n", client_num);
      if(client_num == controllerClient) controllerClient = 255;
      break;
    }
    case WStype_TEXT: {
      String msg = String((char*)payload);
      msg.trim();

      if(msg == "WEB_IMG") {
        webSocket.sendTXT(client_num, "IMG_" + base64Image); // send current image only to the requester
        return; // skip the controller check
      }

      if(client_num == controllerClient) {
        if(msg == "UP") Serial.println(".W");
        else if(msg == "DOWN") Serial.println(".S");
        else if(msg == "LEFT") Serial.println(".A");
        else if(msg == "RIGHT") Serial.println(".D");
        else if(msg == "CAPTURE_IMG") Serial.println(".IMAGE");
        else if(msg == "START_TEL") Serial.println(".START");
        else if(msg == "STOP_TEL") Serial.println(".STOP");
        else if(msg == "LORA_SHORT") Serial.println(".FORCESHORT");
        else if(msg == "LORA_MEDIUM") Serial.println(".FORCEMID");
        else if(msg == "LORA_LONG") Serial.println(".FORCELONG");
      } else {
        Serial.printf("Client %u tried to send command but is not controller\n", client_num);
      }
      break;
    }
    default:
      break;
  }
}

// ====== LoRa telemetry reception ======
void receiveLoRaTelemetry() {
  int packetSize = LoRa.parsePacket();
  if(packetSize) {
    String incoming = "";
    while (LoRa.available()) {
      incoming += (char)LoRa.read();
    }
    incoming.trim();

    int rssi = LoRa.packetRssi();   // RSSI de mensaje entrante
    int avgrssi = -40;              // simulated avg RSSI of -40

    // If the message contains a comma, treat it as CSV
    if (incoming.indexOf(',') >= 0) {
      telemetryCSV = String(rssi) + "," + String(avgrssi) + "," + incoming;  // proper CSV
      Serial.println("CSV: " + telemetryCSV);

      // Broadcast CSV to all WebSocket clients
      webSocket.broadcastTXT(telemetryCSV);
    } else {
      // Normal non-CSV message
      Serial.println("Incoming: " + incoming);
      // You can optionally broadcast this as well if needed
      // webSocket.broadcastTXT(incoming);
    }
  }
}
