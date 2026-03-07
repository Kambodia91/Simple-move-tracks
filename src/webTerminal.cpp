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
#include <ArduinoJson.h>

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

<title>LawnMower Control</title>

<script src="https://cdn.jsdelivr.net/npm/chart.js"></script>

<style>

body{
background:#111;
color:#0f0;
font-family:monospace;
margin:20px;
}

.panel{
border:1px solid #0f0;
padding:15px;
margin-bottom:20px;
}

button{
background:#000;
color:#0f0;
border:1px solid #0f0;
padding:8px;
margin:5px;
}

input{
background:#000;
color:#0f0;
border:1px solid #0f0;
}

#terminal{
height:200px;
overflow-y:scroll;
border:1px solid #0f0;
padding:10px;
}

</style>

</head>

<body>

<h2>Panel sterowania</h2>

<div class="panel">

<h3>Dane</h3>

RPM: <span id="rpm">0</span><br>
Temperatura: <span id="temp">0</span> C<br>
Napicie: <span id="volt">0</span> V<br>
Kat: <span id="angle">0</span><br>

</div>

<div class="panel">

<h3>Sterowanie</h3>

<button onclick="startEngine()">START SILNIKA</button>
<button onclick="stopEngine()">STOP SILNIKA</button>

</div>

<div class="panel">

<h3>PID</h3>

P <input id="p" type="number" step="0.01"><br>
I <input id="i" type="number" step="0.01"><br>
D <input id="d" type="number" step="0.01"><br>

<button onclick="setPID()">USTAW PID</button>

</div>

<div class="panel">

<h3>Wysokosc koszenia</h3>

<input id="height" type="range" min="0" max="100">

<button onclick="setHeight()">Ustaw</button>

</div>

<div class="panel">

<h3>Wykres RPM</h3>

<canvas id="rpmChart"></canvas>

</div>

<div class="panel">

<h3>Terminal</h3>

<div id="terminal"></div>

<input id="cmd">
<button onclick="send()">wyslij</button>

</div>

<script>

let ws = new WebSocket('ws://' + location.hostname + ':81/');

let rpmSpan=document.getElementById("rpm");
let tempSpan=document.getElementById("temp");
let voltSpan=document.getElementById("volt");
let angleSpan=document.getElementById("angle");

let terminal=document.getElementById("terminal");

let rpmData=[];
let labels=[];

let ctx=document.getElementById('rpmChart');

let rpmChart=new Chart(ctx,{
type:'line',
data:{
labels:labels,
datasets:[{
label:'RPM',
data:rpmData
}]
},
options:{
animation:false
}
});

ws.onmessage=function(event){

try{

let data=JSON.parse(event.data);

rpmSpan.innerText=data.rpm;
tempSpan.innerText=data.temp;
voltSpan.innerText=data.volt;
angleSpan.innerText=data.angle;

labels.push("");
rpmData.push(data.rpm);

if(labels.length>50){
labels.shift();
rpmData.shift();
}

rpmChart.update();

}
catch{

terminal.innerHTML+=event.data+"<br>";
terminal.scrollTop=terminal.scrollHeight;

}

}

function startEngine(){

ws.send(JSON.stringify({
cmd:"startEngine"
}));

}

function stopEngine(){

ws.send(JSON.stringify({
cmd:"stopEngine"
}));

}

function setPID(){

let p=document.getElementById("p").value;
let i=document.getElementById("i").value;
let d=document.getElementById("d").value;

ws.send(JSON.stringify({
cmd:"setPID",
p:parseFloat(p),
i:parseFloat(i),
d:parseFloat(d)
}));

}

function setHeight(){

let h=document.getElementById("height").value;

ws.send(JSON.stringify({
cmd:"setCutHeight",
height:parseInt(h)
}));

}

function send(){

let cmd=document.getElementById("cmd").value;

ws.send(cmd);

terminal.innerHTML+="> "+cmd+"<br>";

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

    DynamicJsonDocument doc(256);
    DeserializationError err = deserializeJson(doc, msg);

    if (err) return;

    String cmd = doc["cmd"];

    if (cmd == "startEngine") {

        //startEngine();
        Serial.println("startEngine");
    }

    if (cmd == "stopEngine") {

        //stopEngine();
        Serial.println("stopEngine");
    }

    if (cmd == "setPID") {

        //pidP = doc["p"];
        //pidI = doc["i"];
        //pidD = doc["d"];

        Serial.println("PID updated");

    }

    if (cmd == "setCutHeight") {

        //cuttingHeight = doc["height"];
        Serial.println("setCutHeight update");
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

    doc["rpm"] = rpmMower;
    doc["temp"] = oilTemperature;
    doc["volt"] = Feedback_Serial1.batVoltage;
    doc["angle"] = angle;

    doc["speedL"] = speeds.leftSpeed;
    doc["speedR"] = speeds.rightSpeed;

    String json;
    serializeJson(doc, json);

    webSocket.broadcastTXT(json);
  }
}
