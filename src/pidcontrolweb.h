#pragma once
#include <WebServer.h>
#include <EEPROM.h>

// Variables globales para PID y estado
extern float Kp;
extern float Ki;
extern float Kd;
extern float kv;
extern bool robotGo;
extern long i;
extern uint16_t sensorValues[6]; // O el tamaño que uses
extern const uint8_t SensorCount;


// Referencia al servidor web principal
extern WebServer server;

#define PID_EEPROM_ADDR 0  // Dirección base en EEPROM

void guardarPIDenEEPROM() {
    EEPROM.put(PID_EEPROM_ADDR, Kp);
    EEPROM.put(PID_EEPROM_ADDR + sizeof(float), Ki);
    EEPROM.put(PID_EEPROM_ADDR + 2 * sizeof(float), Kd);
    EEPROM.put(PID_EEPROM_ADDR + 3 * sizeof(float), kv);
    EEPROM.commit();
}

void cargarPIDdeEEPROM() {
    EEPROM.get(PID_EEPROM_ADDR, Kp);
    EEPROM.get(PID_EEPROM_ADDR + sizeof(float), Ki);
    EEPROM.get(PID_EEPROM_ADDR + 2 * sizeof(float), Kd);
    EEPROM.get(PID_EEPROM_ADDR + 3 * sizeof(float), kv);
}

void handlePIDRoot() {
    String html = "<html><head><title>Control PID - Robot</title><style>"
                  "body { font-family: Arial, sans-serif; background: #f4f4f9; text-align: center; margin: 0; padding: 20px; }"
                  "h2 { color: #222; margin-bottom: 20px; }"
                  ".card { background: #fff; padding: 20px; margin: 15px auto; width: 300px; box-shadow: 0 4px 8px rgba(0,0,0,0.1); border-radius: 12px; }"
                  ".slider-container { margin: 15px 0; }"
                  "label { display: block; margin-bottom: 5px; font-weight: bold; }"
                  "input[type=range] { width: 100%; }"
                  ".value { font-size: 14px; margin-top: 5px; }"
                  ".btn { display: inline-block; width: 120px; padding: 12px; margin: 10px; font-size: 16px; font-weight: bold; border: none; border-radius: 8px; cursor: pointer; }"
                  ".go { background-color: #4CAF50; color: white; }"
                  ".stop { background-color: #f44336; color: white; }"
                  "</style></head><body>"
                  "<h2>Configuracion PID y Control</h2>"
                  "<div class='card'>"
                  "<form method='POST' action='/pid'>"
                  "<div class='slider-container'>"
                  "<label for='kp'>Kp</label>"
                  "<input type='range' id='kp' name='kp' min='0' max='1' step='0.0001' value='" + String(Kp, 4) + "' oninput='kp_val.innerText=this.value'>"
                  "<div class='value'>Valor: <span id='kp_val'>" + String(Kp, 4) + "</span></div>"
                  "</div>"
                  "<div class='slider-container'>"
                  "<label for='ki'>Ki</label>"
                  "<input type='range' id='ki' name='ki' min='0' max='0.01' step='0.0001' value='" + String(Ki, 4) + "' oninput='ki_val.innerText=this.value'>"
                  "<div class='value'>Valor: <span id='ki_val'>" + String(Ki, 4) + "</span></div>"
                  "</div>"
                  "<div class='slider-container'>"
                  "<label for='kd'>Kd</label>"
                  "<input type='range' id='kd' name='kd' min='0' max='1' step='0.001' value='" + String(Kd, 4) + "' oninput='kd_val.innerText=this.value'>"
                  "<div class='value'>Valor: <span id='kd_val'>" + String(Kd, 4) + "</span></div>"
                  "</div>"
                  "<div class='slider-container'>"
                  "<label for='kv'>Kv</label>"
                  "<input type='range' id='kv' name='kv' min='0' max='0.2' step='0.0001' value='" + String(kv, 4) + "' oninput='kv_val.innerText=this.value'>"
                  "<div class='value'>Valor: <span id='kv_val'>" + String(kv, 4) + "</span></div>"
                  "</div>"
                  "<input type='submit' value='Guardar PID' class='btn go'>"
                  "</form></div>";

    // Añade los valores QTR aquí
    html += "<div class='card'><h3>Valores Sensores QTR</h3><p id='qtr_values'>Cargando...</p></div>";
    html += R"(
<script>
function updateQTR() {
    fetch('/qtrvalues')
        .then(response => response.json())
        .then(data => {
            document.getElementById('qtr_values').innerText = data.sensors.join(', ');
        });
}
setInterval(updateQTR, 1000);
updateQTR();
</script>
)";

    html += "<div>"
            "<form method='POST' action='/go' style='display:inline;'>"
            "<input type='submit' value='GO' class='btn go'>"
            "</form>"
            "<form method='POST' action='/stop' style='display:inline;'>"
            "<input type='submit' value='STOP' class='btn stop'>"
            "</form>"
            "</div></body></html>";

    server.send(200, "text/html", html);
}

void handlePID() {
    if (server.hasArg("kp")) Kp = server.arg("kp").toFloat();
    if (server.hasArg("ki")) Ki = server.arg("ki").toFloat();
    if (server.hasArg("kd")) Kd = server.arg("kd").toFloat();
    if (server.hasArg("kv")) kv = server.arg("kv").toFloat();
    guardarPIDenEEPROM();
    server.sendHeader("Location", "/pidcontrol");
    server.send(303, "text/plain", "");
}

void handleGo() {
    robotGo = true;
    i = 0; // Reinicia la integral al iniciar el movimiento
    server.sendHeader("Location", "/pidcontrol");
    server.send(303, "text/plain", "");
}

void handleStop() {
    robotGo = false;
    server.sendHeader("Location", "/pidcontrol");
    server.send(303, "text/plain", "");
}
void handleQTRValues() {
    String json = "{ \"sensors\": [";
    for (int i = 0; i < SensorCount; i++) {
        json += String(sensorValues[i]);
        if (i < SensorCount - 1) json += ", ";
    }
    json += "] }";
    server.send(200, "application/json", json);
}

void setupPIDWeb() {
    server.begin();
    server.on("/pidcontrol", handlePIDRoot);
    server.on("/pid", HTTP_POST, handlePID);
    server.on("/go", HTTP_POST, handleGo);
    server.on("/stop", HTTP_POST, handleStop);
    server.on("/qtrvalues", handleQTRValues); // Nueva ruta para valores QTR
}
