#include "fastMath.h"
#include <algorithm>

float normalAtan2(struct Position robot, struct Position ball) {
    float angle = atan2(ball.y - robot.y, ball.x - robot.x);
    return angle;
}

/*
fastCalculate: fastAtan2
calculate mean: normalAtan2

Test in -O2, 100000000 iterations:
    fastCalculate mean is: 40
    calculate mean is: 59
*/
float fastAtan2(float y, float x)
{
  float t0, t1, t3, t4;
  t3 = std::abs(x);
  t1 = std::abs(y);
  t0 = std::max(t3, t1);
  t1 = std::min(t3, t1);
  t3 = float(1) / t0;
  t3 = t1 * t3;

  t4 = t3 * t3;
  t0 =         - float(0.013480470);
  t0 = t0 * t4 + float(0.057477314);
  t0 = t0 * t4 - float(0.121239071);
  t0 = t0 * t4 + float(0.195635925);
  t0 = t0 * t4 - float(0.332994597);
  t0 = t0 * t4 + float(0.999995630);
  t3 = t0 * t3;

  t3 = (abs(y) > abs(x)) ? float(1.570796327) - t3 : t3;
  t3 = (x < 0) ?  float(3.141592654) - t3 : t3;
  t3 = (y < 0) ? -t3 : t3;

  if(std::isnan(t3)) {
      t3 = 0;
  }
  return t3;
}



/*
fastCalculate: this function
calculate mean: sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2))
hypotf32: hypotf32(x2 - x1, y2 - y1)

Test in -O2, 100000000 iterations:
    fastCalculate mean is: 26 nanoseconds
    calculate mean is: 32 nanoseconds
    hypotf32 mean is: 31 nanoseconds
*/
float calculateDistanceBetweenPoints(float a, float b) {
    const float squareX = a*a;
    const float squareY = b*b;
    float sum = squareX + squareY;
    return sqrt(sum);
}



bool sameSign(float a, float b) {
    return a*b >= 0.0f;
}
