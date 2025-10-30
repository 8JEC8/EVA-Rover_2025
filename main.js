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

  // Button event listeners
  document.getElementById('btnUp').addEventListener('click', () => sendCommand('UP'));
  document.getElementById('btnDown').addEventListener('click', () => sendCommand('DOWN'));
  document.getElementById('btnLeft').addEventListener('click', () => sendCommand('LEFT'));
  document.getElementById('btnRight').addEventListener('click', () => sendCommand('RIGHT'));
  document.getElementById('requestImage').addEventListener('click', () => sendCommand('WEB_IMG'));
});