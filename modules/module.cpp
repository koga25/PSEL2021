#include "module.h"

Module::Module(QString networkAddress, quint16 networkPort) {
    _networkAddress = networkAddress;
    _networkPort = networkPort;
    _networkSocket = new QUdpSocket();
}

Module::~Module() {
    // Check if socket is open
    if(_networkSocket->isOpen()) {
        // Close it
        _networkSocket->close();
    }

    // Delete socket pointer
    delete _networkSocket;
}

QString Module::networkAddress() {
    return _networkAddress;
}

quint16 Module::networkPort() {
    return _networkPort;
}

QUdpSocket* Module::networkSocket() {
    return _networkSocket;
}
