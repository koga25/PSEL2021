#include "actuator.h"
#include <math.h>

Actuator::Actuator(QString networkAddress, quint16 networkPort) : Module(networkAddress, networkPort) {
    connectToNetwork();
}

void Actuator::sendCommand(bool isYellow, int robotId, float vx, float vy, float vw, bool enableDribble, float kickPower, bool isChip) {
    // Create grSimPacket
    grSim_Packet packet;

    // Set macro informations
    packet.mutable_commands()->set_isteamyellow(isYellow);
    packet.mutable_commands()->set_timestamp(0.0);

    // Create robot command message
    grSim_Robot_Command *command = packet.mutable_commands()->add_robot_commands();

    // Setting commands
    // Set robot id
    command->set_id(robotId);

    // Set velocity
    command->set_wheelsspeed(false);
    command->set_veltangent(vx);
    command->set_velnormal(vy);
    command->set_velangular(vw);

    // Set kick speed
    if(isChip) {
        command->set_kickspeedx(kickPower * cos(45.0 * (M_PI / 180.0)));
        command->set_kickspeedz(kickPower * sin(45.0 * (M_PI / 180.0)));
    }
    else{
        command->set_kickspeedx(kickPower);
        command->set_kickspeedz(0.0);
    }

    // Set spinner
    command->set_spinner(enableDribble);

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
