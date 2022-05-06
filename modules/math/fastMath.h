#ifndef FASTMATH_H
#define FASTMATH_H

#include <math.h>
#include <algorithm>
#include <modules/structs/structs.h>

//if you want to benchmark the two functions, and compare the outputs from them, remove the comments, in my pc it is 3 times faster.
float normalAtan2(struct Position robot, struct Position ball);

float fastAtan2(float y, float x);

inline float calculateDistanceBetweenPoints(float a, float b);

#endif // FASTMATH_H
