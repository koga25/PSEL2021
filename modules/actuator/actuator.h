#ifndef ACTUATOR_H
#define ACTUATOR_H

#include <modules/module.h>
#include <protobufs/packet.pb.h>

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
    * @param wheelLeft The velocity (in rad/s) of the left wheel
    * @param wheelRight TThe velocity (in rad/s) of the right wheel
    *
    */
    void sendCommand(bool isYellow, int robotId, float wheelLeft, float wheelRight);

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
