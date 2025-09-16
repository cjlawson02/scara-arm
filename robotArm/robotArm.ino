#include "config.h"
#include <Arduino.h>
#include <Stepper.h>
#include <Servo.h>
// #include <VarSpeedServo.h>
#include "pinout.h"
#include "logger.h"
#include "robotGeometry.h"
#include "interpolation.h"
#include "RampsStepper.h"
#include "queue.h"
#include "command.h"
#include "servo_gripper.h"
#include "equipment.h"
#include "endstop.h"

// STEPPER OBJECTS
RampsStepper stepperHigher(X_STEP_PIN, X_DIR_PIN, X_ENABLE_PIN, INVERSE_X_STEPPER);
RampsStepper stepperLower(Y_STEP_PIN, Y_DIR_PIN, Y_ENABLE_PIN, INVERSE_Y_STEPPER);
RampsStepper stepperRotate(Z_STEP_PIN, Z_DIR_PIN, Z_ENABLE_PIN, INVERSE_Z_STEPPER);

// ENDSTOP OBJECTS
Endstop endstopX(X_MIN_PIN, X_DIR_PIN, X_STEP_PIN, X_ENABLE_PIN, X_MIN_INPUT, X_HOME_STEPS, HOME_DWELL);
Endstop endstopY(Y_MIN_PIN, Y_DIR_PIN, Y_STEP_PIN, Y_ENABLE_PIN, Y_MIN_INPUT, Y_HOME_STEPS, HOME_DWELL);
Endstop endstopZ(Z_MIN_PIN, Z_DIR_PIN, Z_STEP_PIN, Z_ENABLE_PIN, Z_MIN_INPUT, Z_HOME_STEPS, HOME_DWELL);
// EQUIPMENT OBJECTS
Servo_Gripper servo_gripper(SERVO_PIN, SERVO_GRIP_DEGREE, SERVO_UNGRIP_DEGREE);
Equipment led(LED_PIN);

RobotGeometry geometry;
Interpolation interpolator;
Queue<Cmd> queue(15);
Command command;

Servo servo_motor;
// int angle = 170;
int angle = 45;
int angle_offset = 0; // offset to compensate deviation from 90 degree(middle position)
// which should gripper should be full closed.

void setup()
{

  SERIALX.begin(BAUD);

  // various pins..
  pinMode(LED_PIN, OUTPUT);

  servo_motor.attach(SERVO_PIN);
  servo_motor.write(angle + angle_offset);
  delay(300);
  servo_motor.detach();

  // reduction of steppers..
  stepperHigher.setReductionRatio((62.0 / 16.0) * (62.0 / 33.0), 200 * 16);
  stepperLower.setReductionRatio(72.0 / 16.0, 200 * 16);
  stepperRotate.setReductionRatio(1.0, 200 * 16);

  // start positions..
  stepperHigher.setPositionRad(PI / 2.0); // 90°
  stepperLower.setPositionRad(0);         // 0°
  stepperRotate.setPositionRad(0);        // 0°
  stepperExtruder.setPositionRad(0);

  stepperHigher.setPositionRad(0);
  stepperLower.setPositionRad(PI / 2.0); // 90°
  stepperRotate.setPositionRad((PI * 2) * GRIPPERFLOATHEIGHT / LEAD);

  // enable and init..
  setStepperEnable(false);

  if (HOME_ON_BOOT)
  { // HOME DURING SETUP() IF HOME_ON_BOOT ENABLED
    homeSequence();
    Logger::logINFO("ROBOT ONLINE");
  }
  else
  {
    setStepperEnable(false); // ROBOT ADJUSTABLE BY HAND AFTER TURNING ON
    if (HOME_X_STEPPER && HOME_Y_STEPPER && !HOME_Z_STEPPER)
    {
      Logger::logINFO("ROBOT ONLINE");
      Logger::logINFO("ROTATE ROBOT TO FACE FRONT CENTRE & SEND G28 TO CALIBRATE");
    }
    if (HOME_X_STEPPER && HOME_Y_STEPPER && HOME_Z_STEPPER)
    {
      Logger::logINFO("ROBOT ONLINE");
      Logger::logINFO("SEND G28 TO CALIBRATE");
    }
    if (!HOME_X_STEPPER && !HOME_Y_STEPPER)
    {
      Logger::logINFO("ROBOT ONLINE");
      Logger::logINFO("HOME ROBOT MANUALLY");
    }
  }

  interpolator.setInterpolation(INITIAL_X, INITIAL_Y, INITIAL_Z, INITIAL_E0, INITIAL_X, INITIAL_Y, INITIAL_Z, INITIAL_E0);

  SERIALX.println("started");
}

void setStepperEnable(bool enable)
{
  stepperRotate.enable(enable);
  stepperLower.enable(enable);
  stepperHigher.enable(enable);
  stepperExtruder.enable(enable);
}

void loop()
{
  // update and Calculate all Positions, Geometry and Drive all Motors...
  interpolator.updateActualPosition();
  geometry.set(interpolator.getXPosmm(), interpolator.getYPosmm(), interpolator.getZPosmm());
  stepperRotate.stepToPositionRad(geometry.getRotRad());
  stepperLower.stepToPositionRad(geometry.getLowRad());
  stepperHigher.stepToPositionRad(geometry.getHighRad());
  stepperExtruder.stepToPositionRad(interpolator.getEPosmm());
  stepperRotate.update();
  stepperLower.update(STEPPERDELAY);
  stepperHigher.update(STEPPERDELAY);

  if (!queue.isFull())
  {
    if (command.handleGcode())
    {
      queue.push(command.getCmd());
      printOk();
    }
  }
  if ((!queue.isEmpty()) && interpolator.isFinished())
  {
    // Serial.println("moreOK");
    executeCommand(queue.pop());
  }

  if (millis() % 500 < 250)
  {
    led.cmdOn();
  }
  else
  {
    led.cmdOff();
  }
}

void cmdMove(Cmd(&cmd))
{
  interpolator.setInterpolation(cmd.valueX, cmd.valueY, cmd.valueZ, cmd.valueE, cmd.valueF);
}
void cmdDwell(Cmd(&cmd))
{
  delay(int(cmd.valueT * 1000));
}

void cmdStepperOn()
{
  setStepperEnable(true);
}
void cmdStepperOff()
{
  setStepperEnable(false);
}

void handleAsErr(Cmd(&cmd))
{
  printComment("Unknown Cmd " + String(cmd.id) + String(cmd.num) + " (queued)");
  printFault();
}

void executeCommand(Cmd cmd)
{
  if (cmd.id == -1)
  {
    String msg = "parsing Error";
    printComment(msg);
    handleAsErr(cmd);
    return;
  }

  if (cmd.valueX == NAN)
  {
    cmd.valueX = interpolator.getXPosmm();
  }
  if (cmd.valueY == NAN)
  {
    cmd.valueY = interpolator.getYPosmm();
  }
  if (cmd.valueZ == NAN)
  {
    cmd.valueZ = interpolator.getZPosmm();
  }
  if (cmd.valueE == NAN)
  {
    cmd.valueE = interpolator.getEPosmm();
  }

  // decide what to do
  if (cmd.id == 'G')
  {
    switch (cmd.num)
    {
    case 0:
      cmdMove(cmd);
      break;
    case 1:
      cmdMove(cmd);
      break;
    case 4:
      cmdDwell(cmd);
      break;
    case 28:
      homeSequence();
      break;
    default:
      handleAsErr(cmd);
    }
  }
  else if (cmd.id == 'M')
  {
    switch (cmd.num)
    {
    case 3:
      servo_gripper.cmdOn();
      break;
    case 5:
      servo_gripper.cmdOff();
      break;
    case 17:
      cmdStepperOn();
      break;
    case 18:
      cmdStepperOff();
      break;
    default:
      handleAsErr(cmd);
    }
  }
  else
  {
    handleAsErr(cmd);
  }
}
void homeSequence()
{
  setStepperEnable(false);
  if (HOME_Y_STEPPER)
  {
    endstopY.home(!INVERSE_Y_STEPPER); // INDICATE STEPPER HOMING DIRECTION
  }
  if (HOME_X_STEPPER)
  {
    endstopX.home(!INVERSE_X_STEPPER); // INDICATE STEPPER HOMING DIRECTION
  }
  if (HOME_Z_STEPPER)
  {
    endstopZ.home(INVERSE_Z_STEPPER); // INDICATE STEPPER HOMING DIRECDTION
  }

  interpolator.setInterpolation(INITIAL_X, INITIAL_Y, INITIAL_Z, INITIAL_E0, INITIAL_X, INITIAL_Y, INITIAL_Z, INITIAL_E0);
  Logger::logINFO("HOMING COMPLETE");
}
