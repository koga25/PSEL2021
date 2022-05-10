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
    fira_message::Ball ball;
    fira_message::Robot robot;
    Position ballPosition;
    Position robotPosition;

    RobotLogic(Actuator *actuator, Vision *vision);

    void makeGoal(bool isYellow, int robotID, bool *fixPosition);
    void walkAroundRadius(bool isYellow, int robotID, double radius, bool *circleAround);

private:
    Actuator *actuator;
    Vision *vision;
    bool checkIfAngleIsWithin(double a, double b, double within);
    double getDistanceToTarget(double a, double b);
    void getBallAndRobotPosition(bool isYellow, int robotID);
    double getAngleOfTargetPosition();
};



#endif // ROBOTLOGIC_H
