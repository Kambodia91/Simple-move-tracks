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

#terminal{
    height:80vh;
    overflow-y:scroll;
    border:1px solid lime;
    padding:10px;
    line-height:0.7;
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

let ws = new WebSocket('ws://' + location.hostname + ':81/');

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