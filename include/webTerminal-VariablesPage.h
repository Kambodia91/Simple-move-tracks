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
</div>

</div>

)rawliteral";