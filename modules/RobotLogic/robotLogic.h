#ifndef ROBOTLOGIC_H
#define ROBOTLOGIC_H
#include <modules/structs/structs.h>
#include <math.h>
#include <modules/math/fastMath.h>
#include <modules/actuator/actuator.h>
#include <modules/vision/vision.h>


class RobotLogic
{
public:
    SSL_DetectionBall ball;
    SSL_DetectionRobot robot;
    Position ballPosition;
    Position robotPosition;

    RobotLogic(Actuator *actuator, Vision *vision);

    void runToBall(bool isYellow, int robotId);
    void walkAroundPosition(bool isYellow, int robotId, bool *circleAround);

private:
    Actuator *actuator;
    Vision *vision;
    bool checkIfAngleIsWithin(double a, double b, double within);
    float getDistanceToTarget(float a, float b);
    void getBallAndRobotPosition(bool isYellow, int robotID);
    float getAngleOfTargetPosition();
};



#endif // ROBOTLOGIC_H
