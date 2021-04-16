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

void Actuator::connectToNetwork() {
    // Connect to simulator (host)
    networkSocket()->connectToHost(networkAddress(), networkPort(), QIODevice::WriteOnly, QAbstractSocket::IPv4Protocol);
}
