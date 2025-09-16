#include "config.h"
#include <Arduino.h>
#include <Stepper.h>
#include <Servo.h>
#include "pinout.h"
#include "logger.h"
#include "robotGeometry.h"
#include "interpolation.h"
#include "RampsStepper.h"
#include "queue.h"
#include "command.h"
#include "servo_gripper.h"
#include "equipment.h"

// STEPPER OBJECTS
RampsStepper stepperHigher(X_STEP_PIN, X_DIR_PIN, X_ENABLE_PIN, INVERSE_X_STEPPER);
RampsStepper stepperLower(Y_STEP_PIN, Y_DIR_PIN, Y_ENABLE_PIN, INVERSE_Y_STEPPER);
RampsStepper stepperRotate(Z_STEP_PIN, Z_DIR_PIN, Z_ENABLE_PIN, INVERSE_Z_STEPPER);

// EQUIPMENT OBJECTS
Servo_Gripper servo_gripper(SERVO_PIN, SERVO_GRIP_DEGREE, SERVO_UNGRIP_DEGREE);
Equipment led(LED_PIN);

RobotGeometry geometry;
Interpolation interpolator;
Queue<Cmd> queue(15);
Command command;

Servo servo_motor;
int angle = 45;
int angle_offset = 0; // offset to compensate deviation from 90 degree(middle position)
// which should gripper should be full closed.

void cmdMove(Cmd(&cmd))
{
  interpolator.setInterpolation(cmd.valueX, cmd.valueY, cmd.valueZ, cmd.valueE, cmd.valueF);
}
void cmdDwell(Cmd(&cmd))
{
  delay(int(cmd.valueT * 1000));
}

void setStepperEnable(bool enable)
{
  stepperRotate.enable(enable);
  stepperLower.enable(enable);
  stepperHigher.enable(enable);
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

void homeSequence()
{
  setStepperEnable(false);

  interpolator.setInterpolation(INITIAL_X, INITIAL_Y, INITIAL_Z, INITIAL_E0, INITIAL_X, INITIAL_Y, INITIAL_Z, INITIAL_E0);
  Logger::logINFO("HOMING COMPLETE");
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

  stepperHigher.setPositionRad(0);
  stepperLower.setPositionRad(PI / 2.0); // 90°
  stepperRotate.setPositionRad((PI * 2) * GRIPPERFLOATHEIGHT / LEAD);

  // enable and init..

  setStepperEnable(false); // ROBOT ADJUSTABLE BY HAND AFTER TURNING ON
  Logger::logINFO("ROBOT ONLINE");
  Logger::logINFO("HOME ROBOT MANUALLY");

  interpolator.setInterpolation(INITIAL_X, INITIAL_Y, INITIAL_Z, INITIAL_E0, INITIAL_X, INITIAL_Y, INITIAL_Z, INITIAL_E0);

  SERIALX.println("started");
}

void loop()
{
  // update and Calculate all Positions, Geometry and Drive all Motors...
  interpolator.updateActualPosition();
  geometry.set(interpolator.getXPosmm(), interpolator.getYPosmm(), interpolator.getZPosmm());
  stepperRotate.stepToPositionRad(geometry.getRotRad());
  stepperLower.stepToPositionRad(geometry.getLowRad());
  stepperHigher.stepToPositionRad(geometry.getHighRad());
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
