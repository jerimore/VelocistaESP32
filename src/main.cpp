#include <Arduino.h>
#include "apwifieeprommode.h"
#include <EEPROM.h>
void setup()
{
Serial.begin(115200);
intentoconexion("espgroup1", "12341243");
}
void loop()
{
loopAP();
}
