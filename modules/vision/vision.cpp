#include "vision.h"

Vision::Vision(QString networkAddress, quint16 networkPort) : Module(networkAddress, networkPort) {
    // Connect to network
    connectToNetwork();

    // Create teams/robots initial map
    for(int i = 0; i <= 1; i++) {
        // Create map for team robots
        QMap<int, fira_message::Robot> *teamDetection = new QMap<int, fira_message::Robot>();

        // Insert team robots map into teams map
        _robotsMap.insert(i, teamDetection);

        // Insert 11 default SSL_DetectionRobot objects into map
        for(int j = 0; j < 11; j++) {
            teamDetection->insert(j, fira_message::Robot());
        }
    }
}

void Vision::processNetworkDatagrams() {
    while(networkSocket()->hasPendingDatagrams()) {
        // Receiving datagram
        QNetworkDatagram datagram = networkSocket()->receiveDatagram();

        // Parsing datagram to protobuf
        fira_message::sim_to_ref::Environment packet;
        packet.ParseFromArray(datagram.data().data(), datagram.data().size());

        // Check if datagram has detection
        if(packet.has_frame()) {
            // Lock mutex to parse data
            _visionMutex.lock();

            // Take detection from packet
            fira_message::Frame detectionFrame = packet.frame();

            // Take ball
            _lastBallDetection = detectionFrame.ball();

            // Check if frame has blue robots
            if(detectionFrame.robots_blue_size() > 0) {
                // Take map for team blue
                QMap<int, fira_message::Robot> *blueMap = _robotsMap.value(false);

                // For each detected robot, insert its detection at map
                for(int i = 0; i < detectionFrame.robots_blue_size(); i++) {
                    // Take detection
                    fira_message::Robot robot = detectionFrame.robots_blue(i);

                    // Insert at map
                    blueMap->insert(robot.robot_id(), robot);
                }
            }

            // Check if frame has yellow robots
            if(detectionFrame.robots_yellow_size() > 0) {
                // Take map for team blue
                QMap<int, fira_message::Robot> *yellowMap = _robotsMap.value(true);

                // For each detected robot, insert its detection at map
                for(int i = 0; i < detectionFrame.robots_yellow_size(); i++) {
                    // Take detection
                    fira_message::Robot robot = detectionFrame.robots_yellow(i);

                    // Insert at map
                    yellowMap->insert(robot.robot_id(), robot);
                }
            }

            // Unlock mutex
            _visionMutex.unlock();
        }

        // Check if datagram has geometry data
        if(packet.has_field()) {
            // Lock mutex to write data
            _visionMutex.lock();

            // Take geometry data
            _lastGeometryData = packet.field();

            // Unlock mutex
            _visionMutex.unlock();
        }
    }
}

fira_message::Robot Vision::getLastRobotDetection(bool isYellow, int playerId) {
    // Lock the mutex to get data (avoid problems with thread sync)
    _visionMutex.lock();
    fira_message::Robot lastRobotDetection = _robotsMap.value(isYellow)->value(playerId);
    _visionMutex.unlock();

    return lastRobotDetection;
}

fira_message::Ball Vision::getLastBallDetection() {
    // Lock the mutex to get data (avoid problems with thread sync)
    _visionMutex.lock();
    fira_message::Ball lastBallDetection = _lastBallDetection;
    _visionMutex.unlock();

    return lastBallDetection;
}

fira_message::Field Vision::getLastGeometryData() {
    // Lock the mutex to get data (avoid problems with thread sync)
    _visionMutex.lock();
    fira_message::Field lastGeometryData = _lastGeometryData;
    _visionMutex.unlock();

    return lastGeometryData;
}

void Vision::connectToNetwork() {
    // Bind network address:port
    networkSocket()->bind(QHostAddress(networkAddress()), networkPort(), QUdpSocket::ShareAddress);

    // Join the multicast group interface
    networkSocket()->joinMulticastGroup(QHostAddress(networkAddress()));
}

