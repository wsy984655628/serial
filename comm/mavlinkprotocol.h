#ifndef MAVLINKPROTOCOL_H
#define MAVLINKPROTOCOL_H

#include <QObject>
#include "Mavlink/v1.0/ardupilotmega/mavlink.h"
#include <QFile>

#include "linkinterface.h"
#include "uas/uasinterface.h"
//#include "linkmanager.h"

class LinkManager;
class MAVLINKProtocol : public QObject
{
    Q_OBJECT
public:
    explicit MAVLINKProtocol();
    ~MAVLINKProtocol();

    void setConnectionManager(LinkManager *manager) {m_connectionManager = manager;}
    void sendMessage(mavlink_message_t msg);
    bool loggingEnabled() { return m_loggingEnabled; }
    void setOnline(bool isonline) { m_isOnline = isonline; }

signals:
    void protocolStatusMessage(const QString& title, const QString& message);
    void valueChanged(const int uasId, const QString& name, const QString& unit, const QVariant& value, const quint64 msec);
    void textMessageReceived(int uasid, int componentid, int severity, const QString& text);
    void receiveLossChanged(int id,float value);
    void messageReceived(mavlink_message_t message);

public slots:
    void receiveBytes( QByteArray b);

private:
    QMap<quint64,mavlink_message_t> m_mavlinkMsgBuffer;
    void handleMessage(quint64 timeid);
    bool m_isOnline;
    int getSystemId() { return 252; }
    int getComponentId() { return 1; }
    bool m_loggingEnabled;
    QFile *m_logfile;

    bool m_throwAwayGCSPackets;
    LinkManager *m_connectionManager;
    bool versionMismatchIgnore;
    QMap<int,qint64> totalReceiveCounter;
    QMap<int,qint64> currReceiveCounter;
    QMap<int,QMap<int,uint8_t> > lastIndex;
    QMap<int,qint64> totalLossCounter;
    QMap<int,qint64> currLossCounter;
    bool m_enable_version_check;
};

#endif // MAVLINKPROTICOL_H
