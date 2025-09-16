#pragma once

class Equipment
{
public:
  Equipment(int equipment_pin);
  void cmdOn();
  void cmdOff();

private:
  int pin;
};
