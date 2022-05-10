#include "fastMath.h"
#include <algorithm>

//if you want to benchmark the two functions, and compare the outputs from them, remove the comments, in my pc it is 3 times faster.
float normalAtan2(struct Position robot, struct Position ball) {
    float angle = atan2(ball.y - robot.y, ball.x - robot.x);
    //const float angle = acos(((robot.x*ball.x) + (robot.y * ball.y))/(sqrt((robot.x * robot.x) + (robot.y * robot.y)) * sqrt((ball.x * ball.x) + (ball.y * ball.y))));
    //std::cout << angle << std::endl;
    return angle;
}

float fastAtan2(float y, float x)
{
  float t0, t1, t2, t3, t4;
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
  //std::cout << t3 << std::endl;
  return t3;
}

float calculateDistanceBetweenPoints(float a, float b) {
    return sqrt(pow(a, 2) + pow(b, 2));
}

bool sameSign(float a, float b) {
    return a*b >= 0.0f;
}
