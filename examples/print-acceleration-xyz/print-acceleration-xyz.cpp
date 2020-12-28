#include <Arduino.h>
#include "qma7981.h"

QMA7981 Accel;

void setup()
{
  Serial.begin(115200);
  Accel.initialize_default();           // setup acceleromter with default settings
  Accel.set_full_scale_range(RANGE_4G); // set maximum range (+- 4G)
  Accel.set_bandwidth(MCLK_DIV_BY_975); // set bandwidth (samples per sec)
                                        // = CLK / DIV_BY = 50k / 975 = 50 samples / sec
}

void loop()
{
  // every 10ms, stream X,Y,Z data to serial port
  delay(10);
  Serial.print(Accel.get_accel_x());
  Serial.print(",");
  Serial.print(Accel.get_accel_y());
  Serial.print(",");
  Serial.print(Accel.get_accel_z());
  Serial.print("\n");
}