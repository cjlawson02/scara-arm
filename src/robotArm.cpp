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
#include <math.h>

static bool motionActive = false;
static uint32_t lastPrint = 0;

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

void cmdMove(const Cmd &cmd)
{

  SERIALX.print("Start move to X=");
  SERIALX.print(cmd.valueX);
  SERIALX.print(" Y=");
  SERIALX.print(cmd.valueY);
  SERIALX.print(" Z=");
  SERIALX.print(cmd.valueZ);
  SERIALX.print(" v=");

  float v = (cmd.valueF > 0) ? (cmd.valueF / 60.0f) : 0.0f; // mm/s from mm/min
  SERIALX.println(v);
  interpolator.setInterpolation(cmd.valueX, cmd.valueY, cmd.valueZ, v);
  motionActive = true; // <-- start driving IK
}

void cmdDwell(const Cmd &cmd)
{
  delay((unsigned long)(cmd.valueT * 1000.0f));
}

void setStepperEnable(bool en)
{
  stepperRotate.enable(en);
  stepperLower.enable(en);
  stepperHigher.enable(en);
  if (en)
  {
    // hold joints
    stepperRotate.stepToPosition(stepperRotate.getPosition());
    stepperLower.stepToPosition(stepperLower.getPosition());
    stepperHigher.stepToPosition(stepperHigher.getPosition());

    // Align logical position to current logical XYZ (so no XY correction)
    interpolator.setCurrentPos(
        interpolator.getXPosmm(),
        interpolator.getYPosmm(),
        interpolator.getZPosmm());

    motionActive = false;
  }
}

void cmdStepperOn()
{
  setStepperEnable(true);
}
void cmdStepperOff()
{
  setStepperEnable(false);
}

void handleAsErr(const Cmd &cmd)
{
  printComment("Unknown Cmd " + String(cmd.id) + String(cmd.num) + " (queued)");
  printFault();
}

void homeSequence()
{
  // 1) Seed joint step counters to your calibrated home steps
  geometry.set(INITIAL_X, INITIAL_Y, INITIAL_Z);
  stepperLower.setPositionRad(geometry.getLowRad());
  stepperHigher.setPositionRad(geometry.getHighRad());
  stepperRotate.setPositionRad(geometry.getRotRad());

  // 2) Seed logical XYZ to the same Cartesian home
  interpolator.setCurrentPos(INITIAL_X, INITIAL_Y, INITIAL_Z);

  // 3) Hold targets = current (no motion after homing)
  stepperHigher.stepToPosition(stepperHigher.getPosition());
  stepperLower.stepToPosition(stepperLower.getPosition());
  stepperRotate.stepToPosition(stepperRotate.getPosition());

  // 4) Stop any active motion gating
  motionActive = false; // and clear any axis flags if you added them

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

  if (isnan(cmd.valueX))
    cmd.valueX = interpolator.getXPosmm();
  if (isnan(cmd.valueY))
    cmd.valueY = interpolator.getYPosmm();
  if (isnan(cmd.valueZ))
    cmd.valueZ = interpolator.getZPosmm();

  SERIALX.print("CMD: ");
  SERIALX.print(cmd.id);
  SERIALX.print(cmd.num);
  SERIALX.print(" X=");
  SERIALX.print(cmd.valueX);
  SERIALX.print(" Y=");
  SERIALX.print(cmd.valueY);
  SERIALX.print(" Z=");
  SERIALX.print(cmd.valueZ);
  SERIALX.print(" F=");
  SERIALX.println(cmd.valueF);

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

  stepperHigher.setPositionRad(0);
  stepperLower.setPositionRad(PI / 2.0); // 90°
  stepperRotate.setPositionRad((PI * 2) * GRIPPERFLOATHEIGHT / LEAD);

  // enable and init..

  setStepperEnable(false); // ROBOT ADJUSTABLE BY HAND AFTER TURNING ON
  Logger::logINFO("ROBOT ONLINE");
  Logger::logINFO("HOME ROBOT MANUALLY");

  interpolator.setInterpolation(INITIAL_X, INITIAL_Y, INITIAL_Z, INITIAL_X, INITIAL_Y, INITIAL_Z);

  SERIALX.println("started");
}

void loop()
{
  if (millis() - lastPrint > 200)
  {
    SERIALX.print("uActive=");
    SERIALX.print(motionActive);
    SERIALX.print(" finished=");
    SERIALX.print(interpolator.isFinished());
    SERIALX.print(" Zcur=");
    SERIALX.print(interpolator.getZPosmm());
    SERIALX.print(" Ztgt=");
    SERIALX.println(command.getCmd().valueZ); // or cache last move target
    lastPrint = millis();
  }

  // 0) Pull and execute command first, if any and allowed
  if (!queue.isEmpty() && interpolator.isFinished())
  {
    executeCommand(queue.pop()); // sets motionActive=true + sets state=0
  }

  // 1) Advance interpolation
  interpolator.updateActualPosition();

  // 2) Gate motionActive only after we've given update() a chance to run
  if (interpolator.isFinished())
  {
    motionActive = false;
  }

  // 3) Feed IK to steppers (or hold)
  geometry.set(interpolator.getXPosmm(), interpolator.getYPosmm(), interpolator.getZPosmm());
  if (motionActive)
  {
    stepperRotate.stepToPositionRad(geometry.getRotRad());
    stepperLower.stepToPositionRad(geometry.getLowRad());
    stepperHigher.stepToPositionRad(geometry.getHighRad());
  }
  else
  {
    stepperRotate.stepToPosition(stepperRotate.getPosition());
    stepperLower.stepToPosition(stepperLower.getPosition());
    stepperHigher.stepToPosition(stepperHigher.getPosition());
  }

  // 4) Tick steppers
  stepperRotate.update(STEPPERDELAY);
  stepperLower.update(STEPPERDELAY);
  stepperHigher.update(STEPPERDELAY);

  // 5) Ingest serial → push into queue
  if (!queue.isFull() && command.handleGcode())
  {
    queue.push(command.getCmd());
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
