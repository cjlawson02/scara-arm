#include <Arduino.h>
#include "RampsStepper.h"

RampsStepper::RampsStepper(int aStepPin, int aDirPin, int aEnablePin, bool aInverse)
{
  setReductionRatio(1, 3200);
  stepPin = aStepPin;
  dirPin = aDirPin;
  enablePin = aEnablePin;
  inverse = aInverse;
  stepperStepPosition = 0;
  stepperStepTargetPosition = 0;
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(enablePin, OUTPUT);
  disable();
}

void RampsStepper::enable(bool value)
{
  digitalWrite(enablePin, !value);
}

void RampsStepper::disable()
{
  digitalWrite(enablePin, HIGH);
}

bool RampsStepper::isOnPosition() const
{
  return stepperStepPosition == stepperStepTargetPosition;
}

int32_t RampsStepper::getPosition() const
{
  return stepperStepPosition;
}

void RampsStepper::setPosition(int32_t value)
{
  stepperStepPosition = value;
  stepperStepTargetPosition = value;
}

void RampsStepper::stepToPosition(int32_t value)
{
  stepperStepTargetPosition = value;
}

void RampsStepper::stepRelative(int32_t value)
{
  value += stepperStepPosition;
  stepToPosition(value);
}

float RampsStepper::getPositionRad() const
{
  return stepperStepPosition / radToStepFactor;
}

void RampsStepper::setPositionRad(float rad)
{
  setPosition((int32_t)lroundf(rad * radToStepFactor));
}

void RampsStepper::stepToPositionRad(float rad)
{
  stepperStepTargetPosition = (int32_t)lroundf(rad * radToStepFactor);
}

void RampsStepper::stepRelativeRad(float rad)
{
  stepRelative((int32_t)lroundf(rad * radToStepFactor));
}

void RampsStepper::update(uint16_t aDelayUs)
{
  int32_t delta = stepperStepTargetPosition - stepperStepPosition;
  if (delta == 0)
    return;

  // compute DIR (high = forward). XOR with inverse to flip if needed.
  bool dirHigh = ((delta > 0) ? true : false) ^ inverse;

  // set DIR once; if it changed, give it setup time before first STEP
  if (dirHigh != lastDirHigh_)
  {
    digitalWrite(dirPin, dirHigh);
    delayMicroseconds(2); // DIR setup (A4988/DRV8825 need ~200ns)
    lastDirHigh_ = dirHigh;
  }
  else
  {
    digitalWrite(dirPin, dirHigh);
  }

  uint32_t now = micros();
  if ((int32_t)(now - nextStepAtUs_) < 0)
    return; // not time yet

  // STEP pulse (min high/low ≥ ~1–2 µs)
  digitalWrite(stepPin, HIGH);
  delayMicroseconds(2);
  digitalWrite(stepPin, LOW);

  stepperStepPosition += (delta > 0) ? 1 : -1;

  // schedule next step based on time *after* the pulse
  nextStepAtUs_ = micros() + aDelayUs;
}

void RampsStepper::setReductionRatio(float gearRatio, int stepsPerRev)
{
  radToStepFactor = gearRatio * stepsPerRev / 2 / PI;
};
