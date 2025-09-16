#pragma once
#include <Arduino.h>
#include <AccelStepper.h>

class RampsStepper
{
public:
  RampsStepper(int stepPin, int dirPin, int enPin, bool inverseDir, float gearRatio, int stepsPerRev,
               bool enableActiveLow = true)
      : stepPin_(stepPin), dirPin_(dirPin), enPin_(enPin),
        enableActiveLow_(enableActiveLow),
        stepper_(AccelStepper::DRIVER, stepPin, dirPin)
  {

    pinMode(enPin_, OUTPUT);
    // default disabled
    digitalWrite(enPin_, enableActiveLow_ ? HIGH : LOW);

    stepsPerRad_ = (gearRatio * stepsPerRev) / (2.0f * PI);
    stepper_.setPinsInverted(inverseDir, false, false);
    stepper_.setMaxSpeed(3000); // safe defaults
    stepper_.setAcceleration(8000);
  }

  // Enable/disable driver
  void enable(bool on = true)
  {
    digitalWrite(enPin_, on ? (enableActiveLow_ ? LOW : HIGH)
                            : (enableActiveLow_ ? HIGH : LOW));
  }
  void disable() { enable(false); }

  // Position (steps)
  int32_t getPosition() { return stepper_.currentPosition(); }
  void setPosition(int32_t s) { stepper_.setCurrentPosition(s); }
  bool isOnPosition() { return stepper_.distanceToGo() == 0; }

  // Commands in steps
  void stepToPosition(int32_t s) { stepper_.moveTo(s); }
  void stepRelative(int32_t ds) { stepper_.move(ds); }

  // Position (radians)
  float getPositionRad() { return getPosition() / stepsPerRad_; }
  void setPositionRad(float rad) { setPosition(lroundf(rad * stepsPerRad_)); }
  void stepToPositionRad(float rad) { stepToPosition(lroundf(rad * stepsPerRad_)); }
  void stepRelativeRad(float rad) { stepRelative(lroundf(rad * stepsPerRad_)); }

  // Call frequently in loop()
  void update() { stepper_.run(); }

private:
  int stepPin_, dirPin_, enPin_;
  bool enableActiveLow_;
  float stepsPerRad_ = 3200.0f / (2.0f * PI);

  AccelStepper stepper_;
};
