#include "linkmanager.h"
#include "uas/uasmanager.h"
#include "SerialCommunication/serialsettingdialog.h"

#include <QTimer>
#include <QPointer>

LinkManager* LinkManager::instance()
{
    static LinkManager* _instance = 0;
    if(_instance == 0)
    {
        _instance = new LinkManager();
    }
    return _instance;
}

LinkManager::LinkManager(QObject *parent) : QObject(parent)
{
//    m_mavlinkDecoder = new MAVLinkDecoder(this);
    m_mavlinkProtocol = new MAVLINKProtocol();
    m_mavlinkProtocol->setConnectionManager(this);
//    connect(m_mavlinkProtocol,SIGNAL(messageReceived(mavlink_message_t)),m_mavlinkDecoder,SLOT(receiveMessage(mavlink_message_t)));
    connect(m_mavlinkProtocol,SIGNAL(messageReceived(mavlink_message_t)),this,SLOT(receiveMessage(mavlink_message_t)));
}

LinkManager::~LinkManager()
{
    m_mavlinkProtocol->setConnectionManager(NULL);
    delete m_mavlinkProtocol;
    m_mavlinkProtocol = NULL;
}

MAVLINKProtocol* LinkManager::getProtocol() const
{
    return m_mavlinkProtocol;
}

void LinkManager::receiveMessage( mavlink_message_t message)
{
    emit messageReceived(message);
}

UASInterface* LinkManager::getUas(int id)
{
    if (m_uasMap.contains(id))
    {
        return m_uasMap.value(id);
    }
    return 0;
}

UASInterface* LinkManager::createUAS(MAVLINKProtocol *mavlink, int sysid, mavlink_heartbeat_t *heartbeat, QObject *parent)
{
    QPointer<QObject> p;

    if(parent != NULL)
    {
        p = parent;
    }
    else
    {
        p = mavlink;
    }

    UASInterface* uas;
    UAS* mav = new UAS(0, sysid);
    qDebug() << heartbeat->type;
    connect(mavlink, SIGNAL(messageReceived(mavlink_message_t)), mav, SLOT(receiveMessage(mavlink_message_t)));
    uas = mav;

    m_uasMap.insert(sysid,uas);

    uas->setAutopilotType((int)heartbeat->autopilot);
    UASManager::instance()->addUAS(uas);
    return uas;
}

