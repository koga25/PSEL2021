#include "robotLogic.h"

RobotLogic::RobotLogic(Actuator *actuator, Vision *vision) {
    this->actuator = actuator;
    this->vision = vision;
}

double RobotLogic::getDistanceToTarget(double a, double b) {
     return calculateDistanceBetweenPoints(a, b);
}

void RobotLogic::getBallAndRobotPosition(bool isYellow, int robotID) {

    getBallPosition();
    getRobotPosition(isYellow, robotID);
}

void RobotLogic::getBallPosition() {
    ball = vision->getLastBallDetection();
    ballPosition = { ball.x(), ball.y() };
}

void RobotLogic::getRobotPosition(bool isYellow, int robotID) {
    robot = vision->getLastRobotDetection(isYellow, robotID);
    robotPosition = { robot.x(), robot.y() };
}

double RobotLogic::getAngleOfTargetPosition(struct Position initial, struct Position final) {
    return fastAtan2(final.y - initial.y, final.x - initial.x);
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
    double angle = getAngleOfTargetPosition(robotPosition, ballPosition);
    const double orientation = robot.orientation();
    double distance = getDistanceToTarget(ballPosition.y - robotPosition.y, ballPosition.x - robotPosition.x);

    //if not circling around and distance > radius, go to ball, else change the angle to be perpendicular to the angle obtained from robot -> ball and then circle the ball.
    if(!(*circleAround) && distance > radius) {
        *circleAround = false;
        angle = angle - orientation;
        angle = fastAtan2(sin(angle), cos(angle));
        actuator->sendCommand(isYellow, robotID, 10*leftWheelVelocity(angle), 10*rightWheelVelocity(angle));
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
        actuator->sendCommand(isYellow, robotID, 10*leftWheelVelocity(fixAngle), 10*rightWheelVelocity(fixAngle));
    } else {
        *fixPosition = false;
        //if robot is in front of the ball and his position.y is highly below or highly over ballPosition.y, try to fix it making the robot go upwards.
        double distance = getDistanceToTarget(ballPosition.y - robotPosition.y, ballPosition.x - robotPosition.x);
        if(fabs(robotPosition.y - ballPosition.y) > 0.12) {
            double angle = fastAtan2(ballPosition.y - robotPosition.y, ballPosition.x - 0.15 - robotPosition.x);
            angle = angle - orientation;
            angle = fastAtan2(sin(angle), cos(angle));
            actuator->sendCommand(isYellow, robotID, 10*leftWheelVelocity(angle), 10*rightWheelVelocity(angle));
        } else {
            //if distance from ball is > 0.1, go towards the ball, else change the angle from robot -> ball to ball -> goal.
            if(distance > 0.1) {

                double angle = getAngleOfTargetPosition(robotPosition, ballPosition);
                angle = angle - orientation;
                angle = fastAtan2(sin(angle), cos(angle));
                actuator->sendCommand(isYellow, robotID, 10*leftWheelVelocity(angle), 10*rightWheelVelocity(angle));
            } else {
                double ballToGoalAngle  = fastAtan2(0 - ballPosition.y, 0.75 - ballPosition.x);
                ballToGoalAngle = ballToGoalAngle - orientation;
                ballToGoalAngle = fastAtan2(sin(ballToGoalAngle), cos(ballToGoalAngle));
                actuator->sendCommand(isYellow, robotID, 10*leftWheelVelocity(ballToGoalAngle), 10*rightWheelVelocity(ballToGoalAngle));
            }
            if(robotPosition.x > ballPosition.x + 0.1) {
                *fixPosition = true;
            }
        }
    }
}

struct BallData RobotLogic::ballVelocityAndDirection() {
    fira_message::Ball tempBall = vision->getLastBallDetection();
    struct BallData ballData = {0, 0, false, false};
    if(tempBall.x() < ball.x()) {
        ballData.isKick = true;
        double xFinal = tempBall.x();
        double xInitial = ball.x();
        double yFinal = tempBall.y();
        double yInitial = ball.y();

        static const double timeElapsed = 1000/60;

        ballData.xVelocity = (xFinal - xInitial)/timeElapsed;
        ballData.yVelocity = (yFinal - yInitial)/timeElapsed;
    }
    ball = tempBall;
    return ballData;
}

bool RobotLogic::checkIfNumberIsWithin(double input1, double input2, double deviation)
{
  return fabs(input1 - input2) <= deviation;
}

//atividade 4. a)
void RobotLogic::predictKick(bool isYellow, int robotID) {
    struct BallData ballData = ballVelocityAndDirection();
    getBallAndRobotPosition(isYellow, robotID);

    if(ballData.isKick) {
        static const double xLimit = -0.680;
        //(S-S0)/V0 = t
        double timeToLimit = ((xLimit - ball.x())/ballData.xVelocity);
        //S = S0 + V0*t
        double yFinalPosition = ballPosition.y + ballData.yVelocity*timeToLimit;
        if(yFinalPosition > 0.225 || yFinalPosition < -0.225) return;

        struct Position finalPosition = {xLimit, yFinalPosition};
        bool xIsWithin = checkIfNumberIsWithin(robotPosition.x, xLimit, 0.04);
        bool yIsWithin = checkIfNumberIsWithin(robotPosition.y, yFinalPosition, 0.04);

        if(!(xIsWithin && yIsWithin)) {
            double angle = getAngleOfTargetPosition(robotPosition, finalPosition);
            angle = angle - robot.orientation();
            angle = fastAtan2(sin(angle), cos(angle));

            actuator->sendCommand(isYellow, robotID, 10*leftWheelVelocity(angle), 10*rightWheelVelocity(angle));


            std::cout << angle << "        ROBOT ORIENTATION: " << robot.orientation()<<"       " <<  std::endl;
        } else {
            std::cout << "robot vx: " << robot.vx() << "       robot vy: " << robot.vy() << std::endl;
            actuator->sendCommand(isYellow, robotID, 0, 0);
        }
    }
}








