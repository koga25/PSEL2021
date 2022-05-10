#ifndef FASTMATH_H
#define FASTMATH_H

#include <math.h>
#include <algorithm>
#include <modules/structs/structs.h>

//if you want to benchmark the two functions, and compare the outputs from them, remove the comments, in my pc it is 3 times faster.
double normalAtan2(struct Position robot, struct Position ball);

double fastAtan2(double y, double x);

double calculateDistanceBetweenPoints(double a, double b);

bool sameSign(double a, double b);

#endif // FASTMATH_H
