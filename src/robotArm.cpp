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

// STEPPER OBJECTS
RampsStepper stepperHigher(X_STEP_PIN, X_DIR_PIN, X_ENABLE_PIN, INVERSE_X_STEPPER, (62.0 / 16.0) * (62.0 / 33.0), 200 * 16);
RampsStepper stepperLower(Y_STEP_PIN, Y_DIR_PIN, Y_ENABLE_PIN, INVERSE_Y_STEPPER, 72.0 / 16.0, 200 * 16);
RampsStepper stepperRotate(Z_STEP_PIN, Z_DIR_PIN, Z_ENABLE_PIN, INVERSE_Z_STEPPER, 1.0, 200 * 16);

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
  float v = (cmd.valueF > 0) ? (cmd.valueF / 60.0f) : 0.0f; // mm/s from mm/min
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
  // 1) ALWAYS tick the steppers first. This is the highest priority.
  stepperRotate.update();
  stepperLower.update();
  stepperHigher.update();

  // 2) If the interpolator is finished, the machine is idle.
  //    We can process serial, pop a command from the queue, and handle LEDs.
  if (interpolator.isFinished())
  {
    // Stop motion gating if it was active
    if (motionActive)
    {
      motionActive = false;
    }

    // Check for and process incoming G-code commands ONLY when idle.
    if (!queue.isFull() && command.handleGcode())
    {
      queue.push(command.getCmd());
    }

    // If there's a command in the queue, execute it.
    if (!queue.isEmpty())
    {
      executeCommand(queue.pop()); // This will set motionActive=true for G0/G1
    }

    // Handle non-time-critical things like LEDs
    if (millis() % 500 < 250)
    {
      led.cmdOn();
    }
    else
    {
      led.cmdOff();
    }
  }

  // 3) If motion is active, update the interpolator and feed new targets to IK.
  //    This block only runs during a move.
  if (motionActive)
  {
    interpolator.updateActualPosition();
    geometry.set(interpolator.getXPosmm(), interpolator.getYPosmm(), interpolator.getZPosmm());
    stepperRotate.stepToPositionRad(geometry.getRotRad());
    stepperLower.stepToPositionRad(geometry.getLowRad());
    stepperHigher.stepToPositionRad(geometry.getHighRad());
  }
}
