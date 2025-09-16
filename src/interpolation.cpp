#include <Arduino.h>
#include "interpolation.h"
#include "config.h"

void Interpolation::setCurrentPos(float px, float py, float pz)
{
  Point p;
  p.xmm = px;
  p.ymm = py;
  p.zmm = pz;
  setCurrentPos(p);
}

void Interpolation::setInterpolation(float px, float py, float pz, float v)
{
  Point p;
  p.xmm = px;
  p.ymm = py;
  p.zmm = pz;
  setInterpolation(p, v);
}

void Interpolation::setInterpolation(float p1x, float p1y, float p1z, float p2x, float p2y, float p2z, float v)
{
  Point p1;
  Point p2;
  p1.xmm = p1x;
  p1.ymm = p1y;
  p1.zmm = p1z;
  p2.xmm = p2x;
  p2.ymm = p2y;
  p2.zmm = p2z;
  setInterpolation(p1, p2, v);
}

void Interpolation::setCurrentPos(const Point &p)
{
  xStartmm = p.xmm;
  yStartmm = p.ymm;
  zStartmm = p.zmm;
  xDelta = yDelta = zDelta = 0;
  // keep reported position in sync
  xPosmm = xStartmm;
  yPosmm = yStartmm;
  zPosmm = zStartmm;
  state = 1;
}

// Prefer using the actual current reported position as p0 when chaining
void Interpolation::setInterpolation(const Point &p1, float v)
{
  Point p0;
  p0.xmm = xPosmm;
  p0.ymm = yPosmm;
  p0.zmm = zPosmm;
  setInterpolation(p0, p1, v);
}

void Interpolation::setInterpolation(const Point &p0, const Point &p1, float av)
{
  float dx = p1.xmm - p0.xmm;
  float dy = p1.ymm - p0.ymm;
  float dz = p1.zmm - p0.zmm;
  float dist = sqrtf(dx * dx + dy * dy + dz * dz);

  if (dist <= 1e-6f)
  {
    xStartmm = p0.xmm;
    yStartmm = p0.ymm;
    zStartmm = p0.zmm;
    xDelta = yDelta = zDelta = 0;
    xPosmm = p1.xmm;
    yPosmm = p1.ymm;
    zPosmm = p1.zmm;
    state = 1;
    return;
  }

  // v: mm/s. Only pick a default if none provided.
  float v = av;
  if (v <= 0.0f)
  {
    // simple default: complete in ~2s regardless of distance
    const float default_duration_s = 2.0f;
    v = dist / default_duration_s;
  }

  tmul = v / dist; // 1/s

  xStartmm = p0.xmm;
  yStartmm = p0.ymm;
  zStartmm = p0.zmm;
  xDelta = dx;
  yDelta = dy;
  zDelta = dz;

  state = 0;
  startTime = micros();
}

void Interpolation::updateActualPosition()
{
  if (state != 0)
    return;

  // wrap-safe delta with uint32_t
  uint32_t now = micros();
  float t = (float)(now - startTime) * 1e-6f; // seconds since start

  // cosine ease-in/out in [0,1]
  float u = t * tmul;
  float progress = -cosf(u * PI) * 0.5f + 0.5f;

  if (u >= 1.0f)
  {
    progress = 1.0f;
    state = 1;
  }

  xPosmm = xStartmm + progress * xDelta;
  yPosmm = yStartmm + progress * yDelta;
  zPosmm = zStartmm + progress * zDelta;
}

bool Interpolation::isFinished() const
{
  return state != 0;
}

float Interpolation::getXPosmm() const
{
  return xPosmm;
}

float Interpolation::getYPosmm() const
{
  return yPosmm;
}

float Interpolation::getZPosmm() const
{
  return zPosmm;
}

Point Interpolation::getPosmm() const
{
  Point p;
  p.xmm = xPosmm;
  p.ymm = yPosmm;
  p.zmm = zPosmm;
  return p;
}
