#include "actuator.h"
#include <math.h>

Actuator::Actuator(QString networkAddress, quint16 networkPort) : Module(networkAddress, networkPort) {
    connectToNetwork();
}

void Actuator::sendCommand(bool isYellow, int robotId, float wheelLeft, float wheelRight) {
    // Create grSimPacket
    fira_message::sim_to_ref::Packet packet;

    // Create robot command message
    fira_message::sim_to_ref::Command *command = packet.mutable_cmd()->add_robot_commands();

    // Setting commands
    // Set robot id
    command->set_id(robotId);
    command->set_yellowteam(isYellow);

    // Set team
    command->set_yellowteam(isYellow);

    // Set wheel speed (rad/s)
    command->set_wheel_left(wheelLeft);
    command->set_wheel_right(wheelRight);

    // Send data to simulator
    std::string buffer;
    packet.SerializeToString(&buffer);

    // Write buffer in network
    networkSocket()->write(buffer.c_str(), buffer.length());
}

void Actuator::circleTheBall(bool isYellow, int robotID, double distanceBetween, double orientation, double angle, double radius, bool* circleOrientation) {
    if(!(*circleOrientation) && distanceBetween > radius) {
        *circleOrientation = false;
        angle = angle - orientation;
        sendCommand(isYellow, robotID, 10*cos(angle), 10*sin(angle));
    } else {
        angle = angle + M_PI_2;
        angle = fastAtan2(sin(angle), cos(angle));
        //angle = angleWrap((angle + M_PI_2), M_PI) - M_PI_2;
        if(!(*circleOrientation) && !checkIfAngleIsWithin(angle, orientation, 0.005)) {
            sendCommand(isYellow, robotID, 1, -1);
        } else {
            *circleOrientation = true;
            angle = orientation;
            if(distanceBetween > radius + 0.1) {
                *circleOrientation = false;
                return;
            }
            //http://www.cs.columbia.edu/~allen/F17/NOTES/icckinematics.pdf
            //ω (R + l/2) = Vr
            //ω (R − l/2) = Vl
            float vr = 1*(radius + 0.042);
            float vl = 1*(radius - 0.042);
            sendCommand(isYellow, robotID, vl*40, vr*40);
        }
    }

}

double Actuator::angleWrap(double a, double b) {
    return a - floor(a/b) * b;
}

bool Actuator::checkIfAngleIsWithin(double a, double b, double within) {
    double diff = a - b;
    const double PI_TIMES_2 = M_PI * 2;
    diff = angleWrap((diff + M_PI_2), M_PI) - M_PI_2;
    if(fabs(diff) <= within || PI_TIMES_2-fabs(diff) <= within) return true;

    return false;
}

void Actuator::makeGoal(bool isYellow, int robotID, double orientation, double angle, bool *fixPosition, Position robotPosition, Position ballPosition) {
    //goal is: x = 0.75; y = 0.0


    angle = angle - orientation;
    angle = fastAtan2(sin(angle), cos(angle));
    std::cout << *fixPosition << std::endl;
    //sendCommand(isYellow, robotID, 10*(cos(angle) + sin(angle)), 10*(cos(angle) - sin(angle)));

    if((robotPosition.x > (ballPosition.x - 0.15)) && *fixPosition) {
        double fixAngle = 0;
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
        sendCommand(isYellow, robotID, 10*(cos(fixAngle) + sin(fixAngle)), 10*(cos(fixAngle) - sin(fixAngle)));
    } else {
        std::cout << "hello" << std::endl;
        *fixPosition = false;
        if(calculateDistanceBetweenPoints(ballPosition.y - robotPosition.y, ballPosition.x - robotPosition.x) > 0.1) {
            sendCommand(isYellow, robotID, 10*(cos(angle) + sin(angle)), 10*(cos(angle) - sin(angle)));
        } else {
            double ballToGoalAngle  = fastAtan2(0 - ballPosition.y, 0.75 - ballPosition.x);
            ballToGoalAngle = ballToGoalAngle - orientation;
            ballToGoalAngle = fastAtan2(sin(ballToGoalAngle), cos(ballToGoalAngle));
            sendCommand(isYellow, robotID, 10*(cos(ballToGoalAngle) + sin(ballToGoalAngle)), 10*(cos(ballToGoalAngle) - sin(ballToGoalAngle)));
            std::cout << "angle is: " << ballToGoalAngle << "    robotPosition = x: " <<robotPosition.x << " y: " <<robotPosition.y << "     ballPosition = x: " << ballPosition.x << " y: " << ballPosition.y << std::endl;
        }

        if(robotPosition.x > ballPosition.x + 0.1) {
            std::cout << "entrou" << std::endl;
            *fixPosition = true;
        }
    }


}

void Actuator::connectToNetwork() {
    // Connect to simulator (host)
    networkSocket()->connectToHost(networkAddress(), networkPort(), QIODevice::WriteOnly, QAbstractSocket::IPv4Protocol);
}
