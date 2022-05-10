#include "robotLogic.h"

RobotLogic::RobotLogic(Actuator *actuator, Vision *vision) {
    this->actuator = actuator;
    this->vision = vision;
}

double RobotLogic::getDistanceToTarget(double a, double b) {
     return calculateDistanceBetweenPoints(a, b);
}

void RobotLogic::getBallAndRobotPosition(bool isYellow, int robotID) {
    ball = vision->getLastBallDetection();
    ballPosition = { ball.x(), ball.y() };
    robot = vision->getLastRobotDetection(isYellow, robotID);
    robotPosition = { robot.x(), robot.y() };
}

double RobotLogic::getAngleOfTargetPosition() {
    return fastAtan2(ballPosition.y - robotPosition.y, ballPosition.x - robotPosition.x);
}

bool RobotLogic::checkIfAngleIsWithin(double a, double b, double within) {
    double diff = a - b;
    const double PI_TIMES_2 = M_PI * 2;
    diff = fastAtan2(sin(diff), cos(diff));
    if(fabs(diff) <= within || PI_TIMES_2-fabs(diff) <= within) return true;

    return false;
}

//Atividade 2. a)
void RobotLogic::walkAroundRadius(bool isYellow, int robotID, double radius, bool *circleAround) {
    getBallAndRobotPosition(isYellow, robotID);
    double angle = getAngleOfTargetPosition();
    const double orientation = robot.orientation();
    double distance = getDistanceToTarget(ballPosition.y - robotPosition.y, ballPosition.x - robotPosition.x);

    //if not circling around and distance > radius, go to ball, else change the angle to be perpendicular to the angle obtained from robot -> ball and then circle the ball.
    if(!(*circleAround) && distance > radius) {
        *circleAround = false;
        angle = angle - orientation;
        angle = fastAtan2(sin(angle), cos(angle));
        actuator->sendCommand(isYellow, robotID, 10*(cos(angle) + sin(angle)), 10*(cos(angle) - sin(angle)));
    } else {
        angle = angle - M_PI_2;
        angle = fastAtan2(sin(angle), cos(angle));
        //angle = angleWrap((angle + M_PI_2), M_PI) - M_PI_2;
        if(!(*circleAround) && (!sameSign(orientation, angle) || !checkIfAngleIsWithin(angle, orientation, 0.015))) {
            actuator->sendCommand(isYellow, robotID, 3, -3);
        } else {
            *circleAround = true;
            angle = orientation;
            if(distance > radius + 0.1) {
                *circleAround = false;
                return;
            }
            //http://www.cs.columbia.edu/~allen/F17/NOTES/icckinematics.pdf
            //ω (R + l/2) = Vr
            //ω (R − l/2) = Vl
            double vr = 1*(radius + 0.042);
            double vl = 1*(radius - 0.042);
            actuator->sendCommand(isYellow, robotID, vl*40, vr*40);
        }
    }
}

//Atividade 2. b)
void RobotLogic::makeGoal(bool isYellow, int robotID, bool *fixPosition) {
    //goal is: x = 0.75; y = 0.0

    getBallAndRobotPosition(isYellow, robotID);

    const double orientation = robot.orientation();


    //if robot is behind ball, walk to front of the ball.
    if((robotPosition.x > (ballPosition.x - 0.15)) && *fixPosition) {
        double fixAngle = 0;
        //if robot is very close to the destined fix position, remove subtract value from destined fix position.y
        if(robotPosition.x <= ballPosition.x - 0.1) {
            fixAngle = fastAtan2(ballPosition.y - robotPosition.y, ballPosition.x - 0.15 - robotPosition.x);
        } else {
            if(ballPosition.y - robotPosition.y > 0) {
                fixAngle = fastAtan2(ballPosition.y - 0.1 - robotPosition.y, ballPosition.x - 0.15 - robotPosition.x);
            } else {
                fixAngle = fastAtan2(ballPosition.y + 0.1 - robotPosition.y, ballPosition.x - 0.15 - robotPosition.x);
            }
        }
        fixAngle = fixAngle - orientation;
        fixAngle = fastAtan2(sin(fixAngle), cos(fixAngle));
        actuator->sendCommand(isYellow, robotID, 10*(cos(fixAngle) + sin(fixAngle)), 10*(cos(fixAngle) - sin(fixAngle)));
    } else {
        *fixPosition = false;
        //if robot is in front of the ball and his position.y is highly below or highly over ballPosition.y, try to fix it making the robot go upwards.
        double distance = getDistanceToTarget(ballPosition.y - robotPosition.y, ballPosition.x - robotPosition.x);
        if(fabs(robotPosition.y - ballPosition.y) > 0.12) {
            double angle = fastAtan2(ballPosition.y - robotPosition.y, ballPosition.x - 0.15 - robotPosition.x);
            angle = angle - orientation;
            angle = fastAtan2(sin(angle), cos(angle));
            actuator->sendCommand(isYellow, robotID, 10*(cos(angle) + sin(angle)), 10*(cos(angle) - sin(angle)));
        } else {
            //if distance from ball is > 0.1, go towards the ball, else change the angle from robot -> ball to ball -> goal.
            if(distance > 0.1) {

                double angle = getAngleOfTargetPosition();
                angle = angle - orientation;
                angle = fastAtan2(sin(angle), cos(angle));
                actuator->sendCommand(isYellow, robotID, 10*(cos(angle) + sin(angle)), 10*(cos(angle) - sin(angle)));
            } else {
                double ballToGoalAngle  = fastAtan2(0 - ballPosition.y, 0.75 - ballPosition.x);
                ballToGoalAngle = ballToGoalAngle - orientation;
                ballToGoalAngle = fastAtan2(sin(ballToGoalAngle), cos(ballToGoalAngle));
                actuator->sendCommand(isYellow, robotID, 10*(cos(ballToGoalAngle) + sin(ballToGoalAngle)), 10*(cos(ballToGoalAngle) - sin(ballToGoalAngle)));
            }
            if(robotPosition.x > ballPosition.x + 0.1) {
                *fixPosition = true;
            }
        }

    }
}








