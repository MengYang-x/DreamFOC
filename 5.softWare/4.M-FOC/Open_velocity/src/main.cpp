// FOC开环速度测试
#include <Arduino.h>
#include "FOC.h"

FOC foc;
void setup()
{
  Serial.begin(115200);
}

void loop()
{
  foc.Open_velocity();
}
