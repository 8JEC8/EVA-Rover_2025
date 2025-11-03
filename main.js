document.addEventListener('DOMContentLoaded', () => {
  const ws = new WebSocket('ws://192.168.4.1:81');

  let myClientId = null;
  let currentController = 255; // 255 = none

  const attemptC = document.getElementById('attemptC');
  const releaseC = document.getElementById('releaseC');
  const buttons = document.querySelectorAll('button');
  const cancelImage = document.getElementById('cancelImage');

  // --- WebSocket Events ---
  ws.onopen = () => {
    console.log("WebSocket connected");
    ws.send("HELLO_" + myClientId); // Let the ESP know your ID
  };

  ws.onmessage = (event) => {
    const msg = event.data;
    console.log("Message received:", msg);

    if (msg.startsWith("ASSIGN_ID_")) {
    myClientId = parseInt(msg.split("_")[2]);
    console.log("Assigned client ID:", myClientId);
    return;
    }

    // --- Control state update ---
    if (msg.startsWith("CTRL_")) {
      currentController = parseInt(msg.split("_")[1]);
      updateControlState(currentController, myClientId);
      return;
    }

    if (msg === "IMG_START") {
    // Disable all buttons except control ones
      buttons.forEach(btn => {
        if (btn !== attemptC && btn !== releaseC && btn !== cancelImage) {
          btn.classList.add("gray");
          btn.disabled = true;
        }
      });
      return;
    }

    if (msg === "IMG_DONE") {
      // Re-enable all buttons except control ones
      buttons.forEach(btn => {
        if (btn !== attemptC && btn !== releaseC && btn !== cancelImage) {
          btn.classList.remove("gray");
          btn.disabled = false;
        }
      });
      return;
    }

    // --- Image handling ---
    if (msg.startsWith("IMG_")) {
      const b64 = msg.substring(4);
      document.getElementById('camImage').src = "data:image/jpeg;base64," + b64;
      return;
    }

    // --- Telemetry handling ---
    const data = msg.split(',');
    if (data.length === 24) {
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
    }
  };

  // --- Send commands ---
  function sendCommand(cmd) {
    if (ws.readyState === WebSocket.OPEN) {
      ws.send(cmd);
      console.log("Command sent:", cmd);
    } else {
      console.warn("WebSocket not open, cannot send:", cmd);
    }
  }

  // --- Control buttons behavior ---
// Buttons keep their default colors (green/red) in HTML
attemptC.onclick = () => sendCommand("REQUEST_CONTROL");
releaseC.onclick = () => sendCommand("RELEASE_CONTROL");

function updateControlState(currentController, myClientId) {
  // Reset both buttons first
  attemptC.classList.remove("gray");
  releaseC.classList.remove("gray");

  attemptC.disabled = false;
  releaseC.disabled = false;

  if (currentController === myClientId) {
    // You have control
    attemptC.classList.add("gray"); // Can't request control again
    attemptC.disabled = true;
    // Release button stays red and active
  } else if (currentController === 255) {
    // No one has control
    // Attempt button stays green and active
    releaseC.classList.add("gray"); // Can't release if no one has control
    releaseC.disabled = true;
  } else {
    // Someone else has control
    attemptC.classList.add("gray");
    releaseC.classList.add("gray");
    attemptC.disabled = true;
    releaseC.disabled = true;
  }
}

  // --- Arrow buttons ---
  document.getElementById('btnUp').addEventListener('click', () => sendCommand('UP'));
  document.getElementById('btnDown').addEventListener('click', () => sendCommand('DOWN'));
  document.getElementById('btnLeft').addEventListener('click', () => sendCommand('LEFT'));
  document.getElementById('btnRight').addEventListener('click', () => sendCommand('RIGHT'));

  // --- Image buttons ---
  document.getElementById('requestImage').addEventListener('click', () => sendCommand('WEB_IMG'));
  document.getElementById('captureImage').addEventListener('click', () => sendCommand('CAPTURE_IMG'));
  document.getElementById('cancelImage').addEventListener('click', () => sendCommand('CANCEL_IMG'));

  // --- Telemetry buttons ---
  document.getElementById('recvTel').addEventListener('click', () => sendCommand('START_TEL'));
  document.getElementById('stopTel').addEventListener('click', () => sendCommand('STOP_TEL'));

  // --- LoRa buttons ---
  document.getElementById('forceShort').addEventListener('click', () => sendCommand('LORA_SHORT'));
  document.getElementById('forceMid').addEventListener('click', () => sendCommand('LORA_MEDIUM'));
  document.getElementById('forceLong').addEventListener('click', () => sendCommand('LORA_LONG'));
});
