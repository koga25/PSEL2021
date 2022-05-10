#include "robotLogic.h"

RobotLogic::RobotLogic(Actuator *actuator, Vision *vision) {
    this->actuator = actuator;
    this->vision = vision;
}

float RobotLogic::getDistanceToTarget(float a, float b) {
     return calculateDistanceBetweenPoints(a, b);
}

void RobotLogic::getBallAndRobotPosition(bool isYellow, int robotID) {
    ball = vision->getLastBallDetection();
    ballPosition = { ball.x(), ball.y() };
    robot = vision->getLastRobotDetection(isYellow, robotID);
    robotPosition = { robot.x(), robot.y() };
}

float RobotLogic::getAngleOfTargetPosition() {
    return fastAtan2(ballPosition.y - robotPosition.y, ballPosition.x - robotPosition.x);
}

bool RobotLogic::checkIfAngleIsWithin(double a, double b, double within) {
    double diff = a - b;
    const double PI_TIMES_2 = M_PI * 2;
    diff = fastAtan2(sin(diff), cos(diff));
    if(fabs(diff) <= within || PI_TIMES_2-fabs(diff) <= within) return true;

    return false;
}

//Atividade 1. a)
void RobotLogic::walkAroundPosition(bool isYellow, int robotId, bool *circleAround) {
    getBallAndRobotPosition(isYellow, robotId);

    float angle = getAngleOfTargetPosition();
    const float orientation = robot.orientation();
    float distance = getDistanceToTarget(ballPosition.y - robotPosition.y, ballPosition.x - robotPosition.x);

    if(distance > 500 && !(*circleAround)) {
        angle = angle - orientation;
        angle = fastAtan2(sin(angle), cos(angle));
        actuator->sendCommand(isYellow, robotId, 1.0, 0.0, angle*5, false, 5.0);

    } else {
        angle = angle - M_PI_2;
        angle = fastAtan2(sin(angle), cos(angle));
        if(!(*circleAround) && (!sameSign(orientation, angle) || !checkIfAngleIsWithin(angle, orientation, 0.05))) {
            actuator->sendCommand(isYellow, robotId, 0.0, 0.0, -1);
        } else {
            if(distance > 1000) {
                *circleAround = false;
                return;
            }
            *circleAround = true;
            actuator->sendCommand(isYellow, robotId, 0.5, 0.0, 1.5);
        }
    }

}

//Atividade 1. b)
void RobotLogic::runToBall(bool isYellow, int robotId) {
    getBallAndRobotPosition(isYellow, robotId);
    const float orientation = robot.orientation();
    float angle = getAngleOfTargetPosition();
    angle = angle - orientation;
    angle = fastAtan2(sin(angle), cos(angle));
    std::cout << angle << std::endl;
    actuator->sendCommand(isYellow, robotId, 1.0, 0.0, angle*5, true, 5.0);
}








