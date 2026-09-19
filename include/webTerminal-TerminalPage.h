#pragma once
#include <Arduino.h>

const char* terminalPage = R"rawliteral(

<div id="terminalPage" class="tabcontent">

<div id="terminal"></div>

<div class="terminal-controls">
    <input id="cmdInput" type="text" placeholder="Wpisz komendę..." />
    <button onclick="sendCommand()">Wyślij</button>
</div>

</div>

)rawliteral";