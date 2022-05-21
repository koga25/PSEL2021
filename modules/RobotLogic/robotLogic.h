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

    void runToBall(bool isYellow, int robotId, float kickPower);
    void walkAroundPosition(bool isYellow, int robotId, bool *circleAround);
    void dribble(bool isYellow, int robotId, bool isWithBall);

private:
    Actuator *actuator;
    Vision *vision;
    bool checkIfAngleIsWithin(float a, float b, float within);
    float getDistanceToTarget(float a, float b);
    void getBallAndRobotPosition(bool isYellow, int robotID);
    float getAngleOfTargetPosition(struct Position targetPosition);
    float getAngleOfTargetPosition();
    //Position getSpecificRobotPosition();
    Closest getClosestObstacle(bool comparingRobotColor, int comparingRobotId);
};



#endif // ROBOTLOGIC_H
