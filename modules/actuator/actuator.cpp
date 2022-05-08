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
        angle = angleWrap((angle + M_PI_2), M_PI) - M_PI_2;
        if(!(*circleOrientation) && !checkIfAngleIsWithin(angle, orientation, 0.05)) {
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
            sendCommand(isYellow, robotID, vl*20, vr*20);
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

void Actuator::connectToNetwork() {
    // Connect to simulator (host)
    networkSocket()->connectToHost(networkAddress(), networkPort(), QIODevice::WriteOnly, QAbstractSocket::IPv4Protocol);
}
