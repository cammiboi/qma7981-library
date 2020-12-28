#include <Arduino.h>
#include "qma7981.h"

void setup()
{
  qma7981_setup_default();
  Serial.begin(115200);
}

void loop()
{
  delay(1000);
  Serial.print(qma7981_get_accel_x());
  Serial.print(",");
  Serial.print(qma7981_get_accel_y());
  Serial.print(",");
  Serial.print(qma7981_get_accel_z());
  Serial.print("\n");
}