

#ifndef UDPMESSAGE_DEF_H
#define UDPMESSAGE_DEF_H

#include "include/rclInclude.hpp"
#include "QVector"
#include <QObject>
#include <QDataStream>

void UdpSendmsg(QByteArray &dataByteArray, QString udpTopic,const QString &address,quint16 port);

#endif