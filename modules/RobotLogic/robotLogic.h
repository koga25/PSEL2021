#ifndef ROBOTLOGIC_H
#define ROBOTLOGIC_H
#include <modules/structs/structs.h>
#include <math.h>
#include <modules/math/fastMath.h>
#include <modules/actuator/actuator.h>
#include <modules/vision/vision.h>
#include <vector>

#define leftWheelVelocity(a) (cos(a) + sin(a))
#define rightWheelVelocity(a) (cos(a) - sin(a))

class RobotLogic
{
public:
    RobotLogic(Actuator *actuator, Vision *vision);
    void makeGoal(bool isYellow, int robotID, bool *fixPosition);
    void walkAroundRadius(bool isYellow, int robotID, double radius, bool *circleAround);
    void predictKick(bool isYellow, int robotID);

private:
    Actuator *actuator;
    Vision *vision;
    fira_message::Ball ball;
    fira_message::Robot robot;
    Position ballPosition;
    Position robotPosition;
    bool checkIfAngleIsWithin(double a, double b, double within);
    bool checkIfNumberIsWithin(double input1, double input2, double deviation);
    double getDistanceToTarget(double a, double b);
    void getBallAndRobotPosition(bool isYellow, int robotID);
    void getBallPosition();
    void getRobotPosition(bool isYellow, int robotID);
    double getAngleOfTargetPosition(struct Position initial, struct Position final);
    struct BallData ballVelocityAndDirection();
};



#endif // ROBOTLOGIC_H
