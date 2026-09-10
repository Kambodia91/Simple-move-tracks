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
#include "webTerminal-Index.h"
#include "cuttingHeight.h"

#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include <ArduinoJson.h>

WebServer server(80);
WebSocketsServer webSocket = WebSocketsServer(81);
WebTerminal webTerminal;

const char* terminalLogo = R"rawliteral(
     __                           __  ___                          
    / /  ____ __      ______     /  |/  /___ _      _____  _____   
   / /  / __ `/ | /| / / __ \   / /|_/ / __ \ | /| / / _ \/ ___/
  / /__/ /_/ /| |/ |/ / / / /  / /  / / /_/ / |/ |/ /  __/ /       
 /_____|__,_/ |__/|__/_/ /_/  /_/  /_/\____/|__/|__/\___/_/ Ver: 0.6

)rawliteral";

extern uint8_t timeoutFlgSbusRx;
extern uint8_t timeoutFlgSerial_1;
extern uint8_t timeoutFlgSerial_2;

size_t WebTerminal::write(uint8_t c) {

    buffer += (char)c;

    if (c == '\n' || buffer.length() > 120) {
        flush();
    }

    return 1;
}
  
  void WebTerminal::flush() {
    if (buffer.length() > 0) {
      StaticJsonDocument<256> doc;
      doc["type"] = "terminal";
      doc["data"] = buffer;

      String json;
      serializeJson(doc, json);
      webSocket.broadcastTXT(json);
      buffer = "";
    }
  }
    
  unsigned long lastRpmSent = 0;

// Access Point credentials
const char* ssid = "LawnMowerTerminal";
const char* password = "";


void setupWebTerminal() {  
  // Start Access Point
  WiFi.softAP(ssid, password);

  // HTTP server
  server.on("/", []() {
    server.send(200, "text/html", htmlPage);
  });
  server.begin();

  // WebSocket server
  webSocket.begin();
  webSocket.onEvent([](uint8_t num, WStype_t type, uint8_t * payload, size_t length) {

  if (type == WStype_CONNECTED) {
    StaticJsonDocument<512> doc;
    doc["type"] = "terminal";
    doc["data"] = terminalLogo;

    String json;
    serializeJson(doc, json);
    webSocket.sendTXT(num, json);
  }

  if (type == WStype_TEXT) {

    String msg;
    msg.reserve(length + 1);
    msg.concat((const char *)payload, length);

    DynamicJsonDocument doc(256);
    DeserializationError err = deserializeJson(doc, msg);

    if (err) return;

    String cmd = doc["cmd"];

    if (cmd == "startEngine") {

        //startEngine();
        webSocket.broadcastTXT("startEngine");
    }

    if (cmd == "stopEngine") {

        //stopEngine();
        webSocket.broadcastTXT("stopEngine");
    }

    if (cmd == "setPID") {

        //pidP = doc["p"];
        //pidI = doc["i"];
        //pidD = doc["d"];

        webSocket.broadcastTXT("PID updated");

    }

    if (cmd == "setCutHeight") {

        //cuttingHeight = doc["height"];
        webSocket.broadcastTXT("setCutHeight update");
    }
  }
});
}

void loopWebTerminal() {
  server.handleClient();
  webSocket.loop();

  // Wysyłanie rpmMower co 500 ms
  unsigned long now = millis();

  if (now - lastRpmSent > 200) {

    lastRpmSent = now;

    StaticJsonDocument<256> doc;

    doc["type"] = "vars";
    doc["rpm"] = rpmMower;
    doc["current1Master"] = Feedback_Serial1.motor_dc_currMaster / 100.0;
    doc["current1Slave"] = Feedback_Serial1.motor_dc_currSlave / 100.0;
    doc["current2Master"] = Feedback_Serial2.motor_dc_currMaster / 100.0;
    doc["current2Slave"] = Feedback_Serial2.motor_dc_currSlave / 100.0;
    doc["temp"] = oilTemperature;
    doc["voltage"] = Feedback_Serial1.batVoltage;
    doc["volt"] = Feedback_Serial1.batVoltage;
    doc["angle"] = angle;
    doc["enable"] = buttonD;

    doc["speedL"] = speeds.leftSpeed;
    doc["speedR"] = speeds.rightSpeed;
    doc["sbusTimeout"] = timeoutFlgSbusRx;
    doc["uart1Timeout"] = timeoutFlgSerial_1;
    doc["uart2Timeout"] = timeoutFlgSerial_2;

    String json;
    serializeJson(doc, json);

    webSocket.broadcastTXT(json);
  }
}
