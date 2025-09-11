#include <Arduino.h>
#include <QTRSensors.h>
#include <math.h>
#include "apwifieeprommode.h"
#include "pidcontrolweb.h"
// ------------------ Sensores QTR ------------------
QTRSensors qtr;
const uint8_t SensorCount = 8;
const uint8_t sensorPins[SensorCount] = {36, 39, 34, 35, 32, 33, 25, 26};
uint16_t sensorValues[SensorCount];

// ------------------ Pines Motor / Driver TB6612 ------------------
// Motor A
const int PWMA = 27;
const int AIN1 = 14;
const int AIN2 = 12;

// Motor B
const int PWMB = 13;
const int BIN1 = 4;
const int BIN2 = 2;

// Standby
const int PINSTDBY = 15;

// ------------------ PWM ESP32 ------------------
const int freq = 5000;
const int resolution = 8;
const int PWMA_channel = 0;
const int PWMB_channel = 1;

// ------------------ PID Variables ------------------
const int vmin = 80;
const int vmax = 150;
float Kp = 0.015;
float Ki = 0.0003;
float Kd = 0.2;
float kv = 0.002; // constante de velocidad
bool robotGo = false;

int p, d, u, vbase;
long i;
int p_old;

// Prototipo de función
void drive(int L, int R);

bool wifiConectado = false;

void setup() {
  EEPROM.begin(512); // Inicializa EEPROM solo una vez
  wifiConectado = intentoconexion("VelocistaESP32", "12345678");
  setupPIDWeb(); // Inicializa servidor web para PID
  Serial.begin(115200);

  // Configura pines del driver como salida
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PINSTDBY, OUTPUT);
  digitalWrite(PINSTDBY, HIGH); // activa driver

  // Configura PWM para motores
  ledcSetup(PWMA_channel, freq, resolution);
  ledcAttachPin(PWMA, PWMA_channel);
  ledcSetup(PWMB_channel, freq, resolution);
  ledcAttachPin(PWMB, PWMB_channel);

  // Configura sensores QTR
  qtr.setTypeAnalog();
  qtr.setSensorPins(sensorPins, SensorCount);

  // Inicializa variables PID
  p_old = 0;
  i = 0;
  cargarPIDdeEEPROM();
}

void loop() {
  if (!wifiConectado) {
    return;
  }

  server.handleClient();

  if (!robotGo) return;

  qtr.read(sensorValues); // leer sensores

  // Error proporcional ponderado
  p = -7*sensorValues[0] -5*sensorValues[1] -3*sensorValues[2] -1*sensorValues[3]
      +1*sensorValues[4] +3*sensorValues[5] +5*sensorValues[6] +7*sensorValues[7];

  i += p;           // integral
  d = p - p_old;    // derivativo
  p_old = p;

  // anti-windup
  if ((p * i) < 0) i = 0;

  u = Kp*p + Ki*i + Kd*d;                     // salida PID
  vbase = vmin + (vmax - vmin) * exp(-kv*abs(Kp*p)); // velocidad base

  // Ajusta velocidad de ruedas
  drive((int)(vbase + u), (int)(vbase - u));
}

void drive(int L, int R) {
  // Limitar valores
  L = constrain(L, -255, 255);
  R = constrain(R, -255, 255);

  // Motor A
  digitalWrite(AIN1, L < 0);
  digitalWrite(AIN2, L >= 0);
  ledcWrite(PWMA_channel, abs(L));

  // Motor B
  digitalWrite(BIN1, R < 0);
  digitalWrite(BIN2, R >= 0);
  ledcWrite(PWMB_channel, abs(R));
}
