//#include <Arduino.h>

hw_timer_t *tim1 = NULL;
int tim1_IRQ_count = 0;

void ARDUINO_ISR_ATTR onTimer()
{
  Serial.println("haha");
  tim1_IRQ_count++;
  Serial.println(timerAlarmEnabled(tim1));
}

void setup()
{
  Serial.begin(115200);
  tim1 = timerBegin(0, 80, true);
  timerAttachInterrupt(tim1, &onTimer, true);
  timerAlarmWrite(tim1, 1000000, true);
  timerAlarmEnable(tim1);
}

void loop()
{
  if (tim1_IRQ_count > 10)
  {
    Serial.println("count trigger");
    tim1_IRQ_count = 0;
  }
}
