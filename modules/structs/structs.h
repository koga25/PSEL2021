#ifndef STRUCTS_H
#define STRUCTS_H
struct Position {
  float x;
  float y;
};

struct Closest {
    float distance;
    bool isYellow;
    int robotId;
};

#endif // STRUCTS_H
