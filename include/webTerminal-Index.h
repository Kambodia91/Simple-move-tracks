#pragma once

#include <Arduino.h>
#include "webTerminal-TerminalPage.h"
#include "webTerminal-VariablesPage.h"

String htmlPage =
R"rawliteral(

<!DOCTYPE html>
<html>

<head>

<title>ESP32 Web Terminal</title>

<style>

body{
    background:black;
    color:lime;
    font-family:monospace;
    margin:20px;
}

button{
    background:#111;
    color:lime;
    border:1px solid lime;
    padding:10px;
    margin-right:5px;
    cursor:pointer;
}

button:hover{
    background:#222;
}

input{
    background:#111;
    color:lime;
    border:1px solid lime;
    padding:8px 10px;
    min-width:220px;
    font-family:monospace;
}

#terminal{
    height:80vh;
    overflow-y:scroll;
    border:1px solid lime;
    padding:10px;
    line-height:0.9;
    font-size:12px;
    white-space:pre;
}

.vars{
    border:1px solid lime;
    padding:10px;
    width:300px;
    line-height:1.8;
}

.tabcontent{
    margin-top:20px;
}

.terminal-controls{
    display:flex;
    gap:8px;
    margin-top:12px;
    align-items:center;
}

.status-row{
    margin-top:12px;
    display:flex;
    gap:12px;
    align-items:center;
    flex-wrap:wrap;
}

.status-item{
    display:flex;
    align-items:center;
    gap:8px;
    min-width:90px;
}

.status-label{
    font-size:12px;
    color:lime;
}

.status-text{
    font-size:12px;
    font-weight:bold;
    min-width:36px;
    display:inline-block;
}

.status-ok{
    color:#2ecc71;
}

.status-error{
    color:#e74c3c;
}

.led{
    display:inline-block;
    width:14px;
    height:14px;
    border-radius:50%;
    border:2px solid #444;
    background:#222;
    box-shadow: inset 0 0 0 1px rgba(255,255,255,0.1);
}

.led-ok{
    background:#2ecc71;
    border-color:#1e8f52;
    box-shadow: 0 0 8px rgba(46, 204, 113, 0.8);
}

.led-error{
    background:#e74c3c;
    border-color:#b93b2f;
    box-shadow: 0 0 8px rgba(231, 76, 60, 0.9);
}

</style>

</head>

<body>

<h2>ESP32 WEB TERMINAL</h2>

<button onclick="showTab('terminalPage')">Terminal</button>
<button onclick="showTab('variablesPage')">Zmienne</button>

<script>

function showTab(tabName)
{
    let tabs = document.getElementsByClassName("tabcontent");

    for(let i=0;i<tabs.length;i++)
    {
        tabs[i].style.display = "none";
    }

    document.getElementById(tabName).style.display = "block";
}

</script>

)rawliteral"

+ String(terminalPage) +
String(variablesPage) +

R"rawliteral(

<script>

let terminal = document.getElementById("terminal");

function setLedState(id, isError)
{
    let led = document.getElementById(id);
    if(!led) return;

    led.classList.toggle("led-error", !!isError);
    led.classList.toggle("led-ok", !isError);
}

function setStatusText(id, isError)
{
    let el = document.getElementById(id);
    if(!el) return;

    el.textContent = isError ? "TIMEOUT" : "OK";
    el.classList.toggle("status-error", !!isError);
    el.classList.toggle("status-ok", !isError);
}

function sendCommand()
{
    let input = document.getElementById("cmdInput");
    if(!input || !input.value.trim()) return;

    if(typeof ws !== "undefined")
    {
        ws.send(JSON.stringify({ cmd: input.value.trim() }));
        input.value = "";
    }
}

let ws = new WebSocket('ws://' + location.hostname + ':81/');

const cmdInput = document.getElementById("cmdInput");
if(cmdInput)
{
    cmdInput.addEventListener("keydown", function(event) {
        if(event.key === "Enter")
        {
            sendCommand();
        }
    });
}

ws.onmessage = function(event)
{
    try
    {
        let json = JSON.parse(event.data);

        // ---------- ZMIENNE ----------
        if(json.type == "vars")
        {
            document.getElementById("rpm").innerText = json.rpm;
            document.getElementById("temp").innerText = json.temp;
            document.getElementById("voltage").innerText = json.voltage;
            document.getElementById("current1Master").innerText = json.current1Master;
            document.getElementById("current1Slave").innerText = json.current1Slave;
            document.getElementById("current2Master").innerText = json.current2Master;
            document.getElementById("current2Slave").innerText = json.current2Slave;
            document.getElementById("enable").innerText = json.enable;
            document.getElementById("leftStickX").innerText = json.leftStickX;
            document.getElementById("leftStickY").innerText = json.leftStickY;

            setLedState("sbusLed", !!json.sbusTimeout);
            setLedState("uart1Led", !!json.uart1Timeout);
            setLedState("uart2Led", !!json.uart2Timeout);

            setStatusText("sbusState", !!json.sbusTimeout);
            setStatusText("uart1State", !!json.uart1Timeout);
            setStatusText("uart2State", !!json.uart2Timeout);
        }

        // ---------- TERMINAL ----------
        if(json.type == "terminal")
        {
            terminal.innerHTML += json.data + "\n";
            terminal.scrollTop = terminal.scrollHeight;
        }
    }
    catch(e)
    {
        console.log(e);
    }
};

</script>

</body>
</html>

)rawliteral";
