#include "fastMath.h"
#include <algorithm>

//if you want to benchmark the two functions, and compare the outputs from them, remove the comments, in my pc it is 3 times faster.
double normalAtan2(struct Position robot, struct Position ball) {
    double angle = atan2(ball.y - robot.y, ball.x - robot.x);
    //const double angle = acos(((robot.x*ball.x) + (robot.y * ball.y))/(sqrt((robot.x * robot.x) + (robot.y * robot.y)) * sqrt((ball.x * ball.x) + (ball.y * ball.y))));
    //std::cout << angle << std::endl;
    return angle;
}

double fastAtan2(double y, double x)
{
  double t0, t1, t2, t3, t4;
  t3 = std::abs(x);
  t1 = std::abs(y);
  t0 = std::max(t3, t1);
  t1 = std::min(t3, t1);
  t3 = double(1) / t0;
  t3 = t1 * t3;

  t4 = t3 * t3;
  t0 =         - double(0.013480470);
  t0 = t0 * t4 + double(0.057477314);
  t0 = t0 * t4 - double(0.121239071);
  t0 = t0 * t4 + double(0.195635925);
  t0 = t0 * t4 - double(0.332994597);
  t0 = t0 * t4 + double(0.999995630);
  t3 = t0 * t3;

  t3 = (abs(y) > abs(x)) ? double(1.570796327) - t3 : t3;
  t3 = (x < 0) ?  double(3.141592654) - t3 : t3;
  t3 = (y < 0) ? -t3 : t3;

  if(std::isnan(t3)) {
      t3 = 0;
  }
  //std::cout << t3 << std::endl;
  return t3;
}

double calculateDistanceBetweenPoints(double a, double b) {
    return sqrt(pow(a, 2) + pow(b, 2));
}

bool sameSign(double a, double b) {
    return a*b >= 0.0f;
}
