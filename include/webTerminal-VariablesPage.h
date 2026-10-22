#pragma once
#include <Arduino.h>

const char* variablesPage = R"rawliteral(

<div id="variablesPage" class="tabcontent" style="display:none;">

<h3>Zmienne</h3>

<div class="vars">
RPM: <span id="rpm">0</span><br>
Temperatura: <span id="temp">0</span><br>
Napiecie: <span id="voltage">0</span><br>
Kat servomechanizmu: <span id="angle">0</span><br>
Prad silnik 1 master: <span id="current1Master">0</span> A<br>
Prad silnik 1 slave: <span id="current1Slave">0</span> A<br>
Prad silnik 2 master: <span id="current2Master">0</span> A<br>
Prad silnik 2 slave: <span id="current2Slave">0</span> A<br>
Gotowosc silnikow: <span id="enable">0</span><br>
Lewo/Prawo X: <span id="rightStickX">0</span><br>
Przod/Tyl  Y: <span id="leftStickY">0</span><br>
Potencjometr A: <span id="potentiometerValueA">0</span><br>
Potencjometr B: <span id="potentiometerValueB">0</span><br>
Potencjometr C: <span id="potentiometerValueC">0</span><br>
<h4>Prędkość zadana</h4>
Master: <span id="targetMasterRpm">0</span> RPM (<span id="targetMasterKmh">0</span> km/h)<br>
Slave: <span id="targetSlaveRpm">0</span> RPM (<span id="targetSlaveKmh">0</span> km/h)<br>
Średnia: <span id="targetAverageRpm">0</span> RPM (<span id="targetAverageKmh">0</span> km/h)<br>
<h4>Prędkość rzeczywista — koła Master</h4>
Master 1: <span id="actualMaster1Rpm">0</span> RPM (<span id="actualMaster1Kmh">0</span> km/h)<br>
Master 2: <span id="actualMaster2Rpm">0</span> RPM (<span id="actualMaster2Kmh">0</span> km/h)<br>
Średnia prędkość kosiarki: <span id="actualAverageRpm">0</span> RPM (<span id="actualAverageKmh">0</span> km/h)<br>

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
