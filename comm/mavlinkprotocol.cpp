#include "mavlinkprotocol.h"
#include "my_gs.h"
#include "linkmanager.h"
#include <QDebug>

MAVLINKProtocol::MAVLINKProtocol():
    m_isOnline(true),
    m_loggingEnabled(false),
    m_logfile(NULL),
    m_connectionManager(NULL)
{

}

MAVLINKProtocol::~MAVLINKProtocol()
{
    m_connectionManager = NULL;
}

void MAVLINKProtocol::sendMessage(mavlink_message_t msg)
{
    Q_UNUSED(msg);
}

void MAVLINKProtocol::receiveBytes( QByteArray b)
{

    mavlink_message_t message;
    mavlink_status_t status;

    int linkId = 0;

    static int mavlink09Count = 0;
    static int nonmavlinkCount = 0;
    static bool decodeFirstPacket = false;
    static bool warnedUser = false;
    static bool checkedUserNonMavlink = false;
    static bool warnedUserNonMavlink = false;

    for (int position = 0; position < b.size(); position++){
        unsigned int decodeState = mavlink_parse_char(linkId,(uint8_t)(b[position]),&message, &status);
        if((uint8_t)b[position] == 0x55) mavlink09Count++;
        if((mavlink09Count > 100) && !decodeFirstPacket && !warnedUser)
        {
            warnedUser = true;            
            emit protocolStatusMessage("MAVLink Version or Baud Rate Mismatch" , "MAVLink Version or Baud Rate Mismatch");
        }

        if (decodeState == 0 && !decodeFirstPacket)
        {
            nonmavlinkCount++;
            if (nonmavlinkCount > 2000 && !warnedUserNonMavlink)
            {
                //500 bytes with no mavlink message.
                if(!checkedUserNonMavlink)
                {
//                    link->requestReset();
                    nonmavlinkCount = 0;
                    checkedUserNonMavlink = true;                 
                }
                else
                {                     
                    warnedUserNonMavlink = true;
                    emit protocolStatusMessage("Mavlink Baud Rate Mismatch", "Mavlink Baud Rate Mismatch");
                }
            }
        }

        if (decodeState == 1)
        {                             
            decodeFirstPacket = true;

            if(message.msgid == MAVLINK_MSG_ID_PING)
            {
                // process ping requests (tgt_system and tgt_comp must be zero)
                mavlink_ping_t ping;
                mavlink_msg_ping_decode(&message, &ping);                
                if(!ping.target_system && !ping.target_component && m_isOnline)
                {                    
                    mavlink_message_t msg;
                    mavlink_msg_ping_pack(getSystemId(), getComponentId(), &msg, ping.time_usec, ping.seq, message.sysid, message.compid );
                    sendMessage(msg);
                }
            }
        }

        quint64 time = My_GS::groundTimeUsecs();        
        m_mavlinkMsgBuffer[time] = message;
        if (m_isOnline)
        {
            handleMessage(time);
        }
    }
}

void MAVLINKProtocol::handleMessage(quint64 timeid)
{
    mavlink_message_t message = m_mavlinkMsgBuffer.value(timeid);
    unsigned int linkId = 0;
    // ORDER MATTERS HERE!
    // If the matching UAS object does not yet exist, it has to be created
    // before emitting the packetReceived signal

    Q_ASSERT_X(m_connectionManager != NULL, "MAVLinkProtocol::receiveBytes", "error:m_connectionManager == NULL");
    UASInterface* uas = m_connectionManager->getUas(message.sysid);

    if (uas == NULL && message.msgid == MAVLINK_MSG_ID_HEARTBEAT)
    {

        if (message.sysid == getSystemId())
        {
            if (m_throwAwayGCSPackets)
            {
                return;
            }
        }
        mavlink_heartbeat_t heartbeat;
        heartbeat.mavlink_version = 0;
        mavlink_msg_heartbeat_decode(&message, &heartbeat);

        if(m_enable_version_check && heartbeat.mavlink_version != MAVLINK_VERSION)
        {
            if (!versionMismatchIgnore)
            {
                emit protocolStatusMessage(tr("The MAVLink protocol version on the MAV and APM Planner mismatch!"),
                                           tr("It is unsafe to use different MAVLink versions. APM Planner therefore refuses to connect to system %1, which sends MAVLink version %2 (APM Planner uses version %3).").arg(message.sysid).arg(heartbeat.mavlink_version).arg(MAVLINK_VERSION));
                versionMismatchIgnore = true;
            }
            return;
        }
        qDebug() << "creat uas" << uas;
        uas = m_connectionManager->createUAS(this, message.sysid,&heartbeat);
    }

    if(uas != NULL)
    {
        totalReceiveCounter[linkId]++;
        currReceiveCounter[linkId]++;

        uint8_t expectedIndex;

        if(lastIndex.contains(message.sysid))
        {
            if(lastIndex.value(message.sysid).contains(message.compid))
            {
                if (lastIndex.value(message.sysid).value(message.compid) == static_cast<uint8_t>(-1))
                {
                    lastIndex[message.sysid][message.compid] = message.seq;
                    expectedIndex = message.seq;
                }
                else
                {
                    expectedIndex = lastIndex[message.sysid][message.compid] + 1;
                }
            }
            else
            {
                lastIndex[message.sysid].insert(message.compid,message.seq);
                expectedIndex = message.seq;
            }
        }
        else
        {
            lastIndex.insert(message.sysid,QMap<int,uint8_t>());
            lastIndex[message.sysid].insert(message.compid,message.seq);
            expectedIndex = message.seq;
        }

        if (message.seq != expectedIndex)
        {
            int16_t lostMessages = message.seq - expectedIndex;
            if (lostMessages < 0)
            {
                // Usually, this happens in the case of an out-of order packet
                lostMessages = 0;
            }
            else
            {
                // Console generates excessive load at high loss rates, needs better GUI visualization
                //QLOG_DEBUG() << QString("Lost %1 messages for comp %4: expected sequence ID %2 but received %3.").arg(lostMessages).arg(expectedIndex).arg(message.seq).arg(message.compid);
            }
            totalLossCounter[linkId] += lostMessages;
            currLossCounter[linkId] += lostMessages;
        }

        lastIndex[message.sysid][message.compid] = message.seq;

        if (totalReceiveCounter[linkId] % 32 == 0)
        {
            // Calculate new loss ratio
            // Receive loss
            float receiveLoss = (double)currLossCounter[linkId]/(double)(currReceiveCounter[linkId]+currLossCounter[linkId]);
            receiveLoss *= 100.0f;
            currLossCounter[linkId] = 0;
            currReceiveCounter[linkId] = 0;
            emit receiveLossChanged(message.sysid, receiveLoss);
        }

        emit messageReceived(message);
    }

}
