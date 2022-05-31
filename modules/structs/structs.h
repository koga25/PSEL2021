#ifndef STRUCTS_H
#define STRUCTS_H
struct Position {
  double x;
  double y;
};

struct BallData {
    double xVelocity;
    double yVelocity;
    bool isLeft;
    bool isKick;
};

#endif // STRUCTS_H
