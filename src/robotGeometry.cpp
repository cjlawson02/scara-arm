#include "robotGeometry.h"
#include "config.h"
#include <math.h>
#include <Arduino.h>

static inline float lawOfCosines(float a, float b, float c)
{
  const float eps = 1e-6f;
  if (a < eps || b < eps)
    return 0.0f; // or a sentinel/prev value
  float num = (a * a + b * b - c * c);
  float den = 2.0f * a * b;
  float x = num / den;
  if (x > 1.0f)
    x = 1.0f;
  if (x < -1.0f)
    x = -1.0f;
  return acosf(x);
}

bool RobotGeometry::elbow = 0;
RobotGeometry::RobotGeometry()
{
}

void RobotGeometry::set(float axmm, float aymm, float azmm)
{
  xmm = axmm;
  ymm = aymm;
  zmm = azmm;
  calculateGrad();
}

float RobotGeometry::getXmm() const
{
  return xmm;
}

float RobotGeometry::getYmm() const
{
  return ymm;
}

float RobotGeometry::getZmm() const
{
  return zmm;
}

float RobotGeometry::getRotRad() const
{
  return rot;
}

float RobotGeometry::getLowRad() const
{
  return low;
}

float RobotGeometry::getHighRad() const
{
  return high;
}

void RobotGeometry::calculateGrad()
{
  // work on locals; don't mutate stored pose
  float rx = xmm;
  float ry = ymm;

  // planar radius
  float dist = sqrtf(rx * rx + ry * ry);
  const float maxReach = L1 + L2;
  const float eps = 1e-6f;

  if (dist > maxReach)
  {
    dist = maxReach - 1e-3f;
    SERIALX.println("IK overflow->limit");
  }

  // choose elbow configuration (heuristic)
  bool elbowLocal = elbow; // if you keep elbow as a member, start from current
  if (rx > 0 && ry < maxReach)
    elbowLocal = false;
  if (rx > 135)
    elbowLocal = false; // parked region
  if (rx < 0 && ry < maxReach)
    elbowLocal = true;

  // reflect for elbow-up solution
  if (elbowLocal)
    rx = -rx;

  // singularity at origin
  if (dist < eps)
  {
    // Define a sane default; shoulder pointing up
    low = 0.0f;
    high = -(PI - acosf((L1 * L1 + L2 * L2 - 0.0f) / (2.0f * L1 * L2))); // ~-PI
    rot = -(PI * 2.0f) * zmm / LEAD;
    elbow = elbowLocal; // persist choice if you keep elbow stateful
    return;
  }

  float D1 = atan2f(ry, rx);
  float D2 = lawOfCosines(dist, L1, L2); // shoulder correction
  low = D1 + D2 - (PI * 0.5f);           // mechanical offset

  // classical elbow term: elbowAngle = PI - acos((L1^2+L2^2 - r^2)/(2 L1 L2))
  // your 'high' uses the negative of that, consistent with later sign flips
  high = lawOfCosines(L1, L2, dist) - PI;

  if (elbowLocal)
  {
    low = -low;
    high = -high;
  }

  // coupling gear ratio
  const float highGearing = 33.0f / 62.0f;
  high = high + (highGearing * low);

  // (Quadrant adjustments are commented out; if needed, re-enable with care)

  rot = -(PI * 2.0f) * zmm / LEAD;

  // persist elbow configuration if intended
  elbow = elbowLocal;
}
