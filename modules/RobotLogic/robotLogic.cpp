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

float RobotLogic::getAngleOfTargetPosition(struct Position targetPosition) {
    return fastAtan2(targetPosition.y - robotPosition.y, targetPosition.x - robotPosition.x);
}

float RobotLogic::getAngleOfTargetPosition() {
    return fastAtan2(ballPosition.y - robotPosition.y, ballPosition.x - robotPosition.x);
}

bool RobotLogic::checkIfAngleIsWithin(float a, float b, float within) {
    double diff = a - b;
    const float PI_TIMES_2 = M_PI * 2;
    diff = fastAtan2(sin(diff), cos(diff));
    if(fabs(diff) <= within || PI_TIMES_2-fabs(diff) <= within) return true;

    return false;
}

Closest RobotLogic::getClosestObstacle(bool comparingRobotColor, int comparingRobotId) {
    //is there a way to dynamically get the quantity of robots?
    uint8_t quantityOfRobots = 6/2;
    //get yellow robots positions;
    SSL_DetectionRobot robotToCompare = vision->getLastRobotDetection(true, 0);
    float minDistance = 9999;
    uint8_t robotId = 0;
    bool isYellow = true;
    for(uint8_t i = 0; i < quantityOfRobots; i++) {
        if(comparingRobotId == i && comparingRobotColor == true) continue;

        robotToCompare = vision->getLastRobotDetection(true, i);
        if(robotToCompare.x() < robotPosition.x) continue;

        float tempDistance = getDistanceToTarget(robotToCompare.x() - robotPosition.x, robotToCompare.y() - robotPosition.y);
        tempDistance = std::min(minDistance, tempDistance);
        if(tempDistance < minDistance) {
            minDistance = tempDistance;
            robotId = i;
        }
    }

    for(uint8_t i = 0; i < quantityOfRobots; i++) {
        if(comparingRobotId == i && comparingRobotColor == false) continue;

        robotToCompare = vision->getLastRobotDetection(false, i);
        if(robotToCompare.x() < robotPosition.x) continue;

        float tempDistance = getDistanceToTarget(robotToCompare.x() - robotPosition.x, robotToCompare.y() - robotPosition.y);
        tempDistance = std::min(minDistance, tempDistance);
        if(tempDistance < minDistance) {
            minDistance = tempDistance;
            robotId = i;
            isYellow = false;
        }
    }
    Closest closestObstacle = { minDistance, isYellow, robotId };
    return closestObstacle;

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
void RobotLogic::runToBall(bool isYellow, int robotId, float kickPower) {
    getBallAndRobotPosition(isYellow, robotId);
    const float orientation = robot.orientation();
    float angle = getAngleOfTargetPosition();
    angle = angle - orientation;
    angle = fastAtan2(sin(angle), cos(angle));
    std::cout << angle << std::endl;
    actuator->sendCommand(isYellow, robotId, 1.0, 0.0, angle*5, true, kickPower);
}

//atividade 3.
void RobotLogic::dribble(bool isYellow, int robotId, bool isWithBall) {
    getBallAndRobotPosition(isYellow, robotId);
    float distanceToBall = getDistanceToTarget(ballPosition.y - robotPosition.y, ballPosition.x - robotPosition.x);

    if(distanceToBall > 200) {
        runToBall(isYellow, robotId, 0.0);
        return;
    }

    Closest closestObstacle = getClosestObstacle(isYellow, robotId);


    if(closestObstacle.distance < 400) {
        std::cout << "closest distance: " <<closestObstacle.distance << "   robot ID: " << closestObstacle.robotId << "      isYellow" << closestObstacle.isYellow << std::endl;
        SSL_DetectionRobot closestRobot = vision->getLastRobotDetection(closestObstacle.isYellow, closestObstacle.robotId);
        struct Position closestRobotPosition = { closestRobot.x(), closestRobot.y() };
        float closestRobotAngle = getAngleOfTargetPosition(closestRobotPosition);
        float dribbleAngle;
        if(closestRobotAngle > 0) {
            dribbleAngle = closestRobotAngle - M_PI_2;
            dribbleAngle = fastAtan2(sin(dribbleAngle), cos(dribbleAngle));
        } else {
            dribbleAngle = closestRobotAngle + M_PI_2;
            dribbleAngle = fastAtan2(sin(dribbleAngle), cos(dribbleAngle));
        }

        if(!checkIfAngleIsWithin(dribbleAngle, robot.orientation(), 0.1f)) {
            actuator->sendCommand(isYellow, robotId, 1.0f, 0.0f, dribbleAngle*5, true);
        } else {
            actuator->sendCommand(isYellow, robotId, 1.0f, 0.0f, 0.0f, true);
        }

        return;
    }

    struct Position goalPosition = { 4600, 0 };
    float angleToGoal = getAngleOfTargetPosition(goalPosition);
    angleToGoal = angleToGoal - robot.orientation();
    angleToGoal = fastAtan2(sin(angleToGoal), cos(angleToGoal));
    bool isAngleWithin = checkIfAngleIsWithin(angleToGoal, robot.orientation(), 0.1);
    if(!isAngleWithin) {
        //std::cout << "isnt" << std::endl;
        actuator->sendCommand(isYellow, robotId, 1.0f, 0.0f, angleToGoal*5, true);
    }  else {
        //std::cout << "iswithin" << std::endl;
        actuator->sendCommand(isYellow, robotId, 1.0f, 0.0f, 0.0f, true);
    }
}








