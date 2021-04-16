#ifndef ACTUATOR_H
#define ACTUATOR_H

#include <modules/module.h>
#include <protobufs/grSim_Packet.pb.h>

class Actuator : public Module
{
public:
    /**
    *
    * Constructor for Actuator objects
    *
    * @param networkAddress The address to connect into the simulator network
    * @param networkPort The port to connect into the simulator network
    *
    */
    Actuator(QString networkAddress, quint16 networkPort);

    /**
    *
    * Constructor for Actuator objects
    *
    * @param isYellow If the robot that will be commanded is from team yellow
    * @param robotId The id of the robot that the command will be sent
    * @param vx The velocity in x-axis to send to robot
    * @param vy The velocity in y-axis to send to robot
    * @param vw The angular velocity to send to robot
    * @param enableDribble If the robot needs to enable the dribbling device
    * @param kickPower The power of the kick that the kick device will make
    * @param isChip If the kick is parabollic
    *
    */
    void sendCommand(bool isYellow, int robotId, float vx, float vy, float vw, bool enableDribble = false, float kickPower = 0.0f, bool isChip = false);

private:
    /**
    *
    * Override for the connectToNetwork method of Module class.
    * This method connets to the simulator network
    *
    */
    void connectToNetwork();
};

#endif // ACTUATOR_H
