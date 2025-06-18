//------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------ 
#include <Arduino.h>
#include <ArduinoLogger.h>                         // [Serial / Terminal]
#include "defines.h"
#include "config.h"
#include "setup.h"
#include "moveTracks.h"
#include "sendCmd.h"
#include "sbusRx.h"
#include "starter.h"
#include "prm01.h"
#include "controlServo.h"
#include "temperatureDS18B20.h"
#include "webTerminal.h"
#include "cuttingHeight.h"

#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>

WebServer server(80);
WebSocketsServer webSocket = WebSocketsServer(81);
WebTerminal webTerminal;

size_t WebTerminal::write(uint8_t c) {
    buffer += (char)c;
    if (c == '\n') {
      flush();
    }
    return 1;
  }
  
  void WebTerminal::flush() {
    if (buffer.length() > 0) {
      webSocket.broadcastTXT(buffer);
      buffer = "";
    }
  }
    
  unsigned long lastRpmSent = 0;

// Access Point credentials
const char* ssid = "LawnMowerTerminal";
const char* password = "";

// Terminal HTML page
const char* htmlPage = R"rawliteral(
  <!DOCTYPE html>
  <html>
    <head>
      <title>ESP32 Terminal</title>
      <style>
        body { font-family: monospace; background: #000; color: #0f0; margin: 0; padding: 20px; }
        #dataPanel { margin-bottom: 20px; }
        #dataPanel p { margin: 5px 0; }
        #terminal { white-space: pre; border: 1px solid #0f0; padding: 10px; height: 400px; overflow-y: scroll; }
        #inputArea { margin-top: 10px; }
        input { width: 80%; }
      </style>
    </head>
    <body>
      <h2>ESP32 Terminal</h2>
  
      <div id="dataPanel">
        <p><b>Obroty silnika:</b> <span id="rpm">0</span> RPM</p>
        <p><b>Temperatura silnika:</b> <span id="temp">0.0</span> °C</p>
        <p><b>Napięcie baterii:</b> <span id="voltage">0.00</span> V</p>
        <p><b>Kąt skrętu:</b> <span id="angle">0.0</span> °</p>
      </div>
  
      <div id="terminal"></div>
  
      <div id="inputArea">
        <input type="text" id="cmd" placeholder="Wpisz komendę">
        <button onclick="send()">Wyślij</button>
      </div>
  
      <script>
        let terminal = document.getElementById('terminal');
        let rpmDisplay = document.getElementById('rpm');
        let tempDisplay = document.getElementById('temp');
        let voltageDisplay = document.getElementById('voltage');
        let angleDisplay = document.getElementById('angle');
        let ws = new WebSocket('ws://' + location.hostname + ':81/');
  
        ws.onmessage = function(event) {
          if (event.data.startsWith("RPM:")) {
            rpmDisplay.textContent = event.data.substring(4);
          } else if (event.data.startsWith("TEMP:")) {
            tempDisplay.textContent = event.data.substring(5);
          } else if (event.data.startsWith("VOLT:")) {
            voltageDisplay.textContent = event.data.substring(5);
          } else if (event.data.startsWith("ANGLE:")) {
            angleDisplay.textContent = event.data.substring(6);
          } else {
            terminal.innerText += event.data + '\\n';
            terminal.scrollTop = terminal.scrollHeight;
          }
        };
  
        function send() {
          let input = document.getElementById('cmd');
          let text = input.value.trim();
          if (text.length > 0) {
            ws.send(text);
            terminal.innerText += '> ' + text + '\\n';
            terminal.scrollTop = terminal.scrollHeight;
            input.value = '';
          }
        }
      </script>
    </body>
  </html>
  )rawliteral";
  
  
  
void setupWebTerminal() {  
  // Start Access Point
  WiFi.softAP(ssid, password);
  Serial.println("Uruchomiono Access Point: " + WiFi.softAPIP().toString());

  // HTTP server
  server.on("/", []() {
    server.send(200, "text/html", htmlPage);
  });
  server.begin();

  // WebSocket server
  webSocket.begin();
  webSocket.onEvent([](uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
    if (type == WStype_TEXT) {
      String msg = String((char*)payload);
      Serial.println("Odebrano z przeglądarki: " + msg);

      // Wyślij odpowiedź do przeglądarki
      String reply = "ESP32 otrzymał: " + msg;
      webSocket.sendTXT(num, reply);
    }
  });
}

void loopWebTerminal() {
  server.handleClient();
  webSocket.loop();

  // Wysyłanie rpmMower co 500 ms
  unsigned long now = millis();
  if (now - lastRpmSent > 500) {
    lastRpmSent = now;
    webSocket.broadcastTXT("RPM:" + String(rpmMower));
    webSocket.broadcastTXT("TEMP:" + String(oilTemperature));
    webSocket.broadcastTXT("VOLT:" + String(Feedback_Serial1.batVoltage, 2));
    webSocket.broadcastTXT("ANGLE:" + String(angle, 1));
  }
}
