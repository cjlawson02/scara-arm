#ifndef RAMPSSTEPPER_H_
#define RAMPSSTEPPER_H_

class RampsStepper
{
public:
  RampsStepper(int aStepPin, int aDirPin, int aEnablePin, bool aInverse);
  void enable(bool value = true);
  void disable();

  bool isOnPosition() const;
  int32_t getPosition() const;
  void setPosition(int32_t value);
  void stepToPosition(int32_t value);
  void stepRelative(int32_t value);

  float getPositionRad() const;
  void setPositionRad(float rad);
  void stepToPositionRad(float rad);
  void stepRelativeRad(float rad);

  void update(uint16_t aDelay = 40);

  void setReductionRatio(float gearRatio, int stepsPerRev);

private:
  uint32_t nextStepAtUs_ = 0;
  bool lastDirHigh_ = false;
  long stepperStepTargetPosition;
  long stepperStepPosition;

  int stepPin;
  int dirPin;
  int enablePin;
  bool inverse;

  float radToStepFactor;
};

#endif
