#pragma once
#include <Arduino.h>

const char* variablesPage = R"rawliteral(

<div id="variablesPage" class="tabcontent" style="display:none;">

<h3>Zmienne</h3>

<div class="vars">
RPM: <span id="rpm">0</span><br>
Temperatura: <span id="temp">0</span><br>
Napiecie: <span id="voltage">0</span><br>
Prad silnik 1 master: <span id="current1Master">0</span> A<br>
Prad silnik 1 slave: <span id="current1Slave">0</span> A<br>
Prad silnik 2 master: <span id="current2Master">0</span> A<br>
Prad silnik 2 slave: <span id="current2Slave">0</span> A<br>
Gotowosc silnikow: <span id="enable">0</span><br>
Lewy drążek X: <span id="leftStickX">0</span><br>
Lewy drążek Y: <span id="leftStickY">0</span><br>

<div class="status-row">
  <div class="status-item">
    <span class="status-label">SBUS</span>
    <span id="sbusLed" class="led led-ok" title="SBUS timeout"></span>
    <span id="sbusState" class="status-text status-ok">OK</span>
  </div>
  <div class="status-item">
    <span class="status-label">UART1</span>
    <span id="uart1Led" class="led led-ok" title="UART1 timeout"></span>
    <span id="uart1State" class="status-text status-ok">OK</span>
  </div>
  <div class="status-item">
    <span class="status-label">UART2</span>
    <span id="uart2Led" class="led led-ok" title="UART2 timeout"></span>
    <span id="uart2State" class="status-text status-ok">OK</span>
  </div>
</div>
</div>

</div>

)rawliteral";
