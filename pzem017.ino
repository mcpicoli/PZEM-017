#include "src/ModbusMaster/src/ModbusMaster.h"
#include "pzem017.h"

ModbusMaster node;
PZEM017 pzem;

void setup()
{
  Serial2.begin(9600, SERIAL_8N2, 10, 20);
  node.begin(0xF8, Serial2);

  pzem.begin(node, 0xf8);
}

void loop()
{
  Serial.println("Tensão: " + String(pzem.voltage()));
  delay(5000);

}
