#include "udpserver/UdpSend.hpp"

void UdpSendmsg(QByteArray &dataByteArray, QString udpTopic, const QString &address, quint16 port)
{
        UdpMessage udpMessage;
        udpMessage.topic = udpTopic;
        udpMessage.data = std::move(dataByteArray);

        QByteArray msgByteArray;
        QDataStream msgByteArrayStream(&msgByteArray, QIODevice::WriteOnly);
        msgByteArrayStream << udpMessage;

        QUdpSocket udpSocket;
        udpSocket.bind(QHostAddress::Any);                                            // 绑定本地地址和端口号
        int ans = udpSocket.writeDatagram(msgByteArray, QHostAddress(address), port); // 将目标 IP 和端口指定为接收端的地址和端口
        std::cout << "udp signal:"<< ans << '\n';
}