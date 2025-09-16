#pragma once
#include <Arduino.h> // for uint32_t, uint8_t (or use <stdint.h>)
#include <stdint.h>

struct Point
{
  float xmm, ymm, zmm;
  Point(float x = 0, float y = 0, float z = 0, float e = 0)
      : xmm(x), ymm(y), zmm(z) {}
};

class Interpolation
{
public:
  Interpolation()
      : state(1), startTime(0),
        xStartmm(0), yStartmm(0), zStartmm(0),
        xDelta(0), yDelta(0), zDelta(0),
        xPosmm(0), yPosmm(0), zPosmm(0),
        v(0), tmul(0) {}

  void setCurrentPos(float px, float py, float pz);
  void setInterpolation(float px, float py, float pz, float v = 0);
  void setInterpolation(float p1x, float p1y, float p1z,
                        float p2x, float p2y, float p2z, float av = 0);

  void setCurrentPos(const Point &p);
  void setInterpolation(const Point &p1, float v = 0);
  void setInterpolation(const Point &p0, const Point &p1, float v = 0);

  void updateActualPosition();
  bool isFinished() const;

  float getXPosmm() const;
  float getYPosmm() const;
  float getZPosmm() const;
  Point getPosmm() const;

private:
  uint8_t state;      // 0=running, 1=finished/idle
  uint32_t startTime; // micros() wrap-safe

  float xStartmm, yStartmm, zStartmm;
  float xDelta, yDelta, zDelta;
  float xPosmm, yPosmm, zPosmm;
  float v;    // mm/s
  float tmul; // 1/s
};
