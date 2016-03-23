#ifndef LINKMANAGER_H
#define LINKMANAGER_H

#include <QObject>
#include "mavlinkprotocol.h"
#include "mavlinkdecoder.h"
#include "uas/uasinterface.h"
#include "uas/uas.h"

class LinkManager : public QObject
{
    Q_OBJECT
public:
    explicit LinkManager(QObject *parent = 0);
    static LinkManager* instance();
    ~LinkManager();


    MAVLINKProtocol* getProtocol() const;
    UASInterface* getUas(int id);
    UASInterface* createUAS(MAVLINKProtocol* mavlink, int sysid, mavlink_heartbeat_t* heartbeat, QObject* parent=NULL);

signals:
    void protocolStatusMessage(QString title,QString text);

    /** @brief aggregated signal for when link status changes */

    void messageReceived(mavlink_message_t message);

public slots:
    void receiveMessage(mavlink_message_t message);

private:
    QMap<int,LinkInterface*> m_connectionMap;
    QMap<int,UASInterface*> m_uasMap;
    QMap<QString,int> m_portToBaudMap;
    MAVLinkDecoder *m_mavlinkDecoder;
    MAVLINKProtocol *m_mavlinkProtocol;
    QString m_logSubDir;
    bool m_mavlinkLoggingEnabled;
};

#endif // LINKMANAGER_H
