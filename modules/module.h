#ifndef MODULE_H
#define MODULE_H

#include <QUdpSocket>
#include <QNetworkDatagram>

class Module
{
public:
    /**
    *
    * Constructor for Module objects
    *
    * @param networkAddress The address to the network that this socket will connect in
    * @param networkPort The port to the network that this socket will connect in
    *
    */
    Module(QString networkAddress, quint16 networkPort);

    /**
    *
    * Destructor for Module objects. This method will disconnect the related socket and delete its pointer
    *
    */
    virtual ~Module();

protected:
    /**
    *
    * Getter for networkAddress
    *
    * @return QString The network address related to the Module
    *
    */
    QString networkAddress();

    /**
    *
    * Getter for networkPort
    *
    * @return quint16 The network port related to the Module
    *
    */
    quint16 networkPort();

    /**
    *
    * Getter for networkSocket
    *
    * @return QUdpSocket* The pointer for the socket
    *
    */
    QUdpSocket* networkSocket();

private:
    // Network socket
    QUdpSocket *_networkSocket;

    // Network params
    QString _networkAddress;
    quint16 _networkPort;

    /**
    *
    * Virtual method implemented by children to connect
    * to the network using params when object was created by
    * the constructor
    *
    */
    virtual void connectToNetwork() = 0;
};

#endif // MODULE_H
