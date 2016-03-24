
#include "uas.h"
#include "uasmanager.h"
#include "my_gs.h"
#include "comm/gsmavlink.h"
#include "comm/linkmanager.h"

#include <QList>
#include <QMessageBox>
#include <QTimer>
#include <QSettings>
#include <iostream>
#include <QDesktopServices>

#include <cmath>
#include <qmath.h>

#include <QDebug>

UAS::UAS(MAVLINKProtocol* protocol, int id) : UASInterface(),
   uasId(id),
   unknownPackets(),

   name(""),
   type(-1),
   autopilot(-1),
   base_mode(-1),
   // custom_mode not initialized
   custom_mode(-1),
   status(-1),

   onboardTimeOffset(0),

   attitudeStamped(false),
   lastAttitude(0),

   roll(0.0),
   pitch(0.0),
   yaw(0.0),

   // The protected members.
   lastNonNullTime(0)
{
   Q_UNUSED(protocol);

   for (unsigned int i = 0; i<255;++i)
   {
       componentID[i] = -1;
       componentMulti[i] = false;
   }
   systemId = My_GS::defaultSystemId;
   componentId = My_GS::defaultComponentId;
   QTimer *heartbeattimer = new QTimer(this);
   heartbeattimer->start(MAVLINK_HEARTBEAT_DEFAULT_RATE * 1000);
}

/**
* Saves the settings of name, airframe, autopilot type and battery specifications
* by calling writeSettings.
*/
UAS::~UAS()
{
}


int UAS::getUASID() const
{
   return uasId;
}

void UAS::receiveMessage( mavlink_message_t message)
{
   if (message.sysid == uasId && (!attitudeStamped || (attitudeStamped && (lastAttitude != 0)) || message.msgid == MAVLINK_MSG_ID_ATTITUDE))
   {
       switch (message.msgid)
       {
       case MAVLINK_MSG_ID_HEARTBEAT:
       {
           emit heartbeat(this);
           mavlink_heartbeat_t state;
           mavlink_msg_heartbeat_decode(&message, &state);

           // Send the base_mode and system_status values to the plotter. This uses the ground time
           // so the Ground Time checkbox must be ticked for these values to display
           quint64 time = getUnixTime();
           QString name = QString("M%1:HEARTBEAT.%2").arg(message.sysid);
           emit valueChanged(uasId, name.arg("base_mode"), "bits", state.base_mode, time);
           emit valueChanged(uasId, name.arg("custom_mode"), "bits", state.custom_mode, time);
           emit valueChanged(uasId, name.arg("system_status"), "-", state.system_status, time);
           } break;

       case MAVLINK_MSG_ID_ATTITUDE:
       {
           mavlink_attitude_t attitude;
           mavlink_msg_attitude_decode(&message, &attitude);
           quint64 time = getUnixReferenceTime(attitude.time_boot_ms);

           lastAttitude = time;
           setRoll(My_GS::limitAngleToPMPIf(attitude.roll));
           setPitch(My_GS::limitAngleToPMPIf(attitude.pitch));
           setYaw(My_GS::limitAngleToPMPIf(attitude.yaw));

           emit attitudeChanged(this, getRoll(), getPitch(), getYaw(), time);
           emit attitudeRotationRatesChanged(uasId, attitude.rollspeed, attitude.pitchspeed, attitude.yawspeed, time);

           QString name = QString("M%1:GCS Status.%2").arg(message.sysid);
           emit valueChanged(uasId,name.arg("Roll"),"deg",QVariant(getRoll() * (180.0/M_PI)),time);
           emit valueChanged(uasId,name.arg("Pitch"),"deg",QVariant(getPitch() * (180.0/M_PI)),time);
           emit valueChanged(uasId,name.arg("Yaw"),"deg",QVariant(getYaw() * (180.0/M_PI)),time);
           qDebug() << getRoll();
       }
           break;

       default:
       {
           if (!unknownPackets.contains(message.msgid))
           {
               unknownPackets.append(message.msgid);
#ifdef QT_DEBUG // Remove these messages from the release build
               QString errString = tr("UNABLE TO DECODE MESSAGE NUMBER %1").arg(message.msgid);
               emit textMessageReceived(uasId, message.compid, 255, errString);
#endif
//               QLOG_INFO() << "Unable to decode message from system " << message.sysid
//                           << " with message id:" << message.msgid;
           }
       }
           break;
       }
   }
   emit mavlinkMessageRecieved(message);
}

quint64 UAS::getUnixReferenceTime(quint64 time)
{
   // Same as getUnixTime, but does not react to attitudeStamped mode
   if (time == 0)
   {
       //        QLOG_DEBUG() << "XNEW time:" <<My_GS::groundTimeMilliseconds();
       return My_GS::groundTimeMilliseconds();
   }
   // Check if time is smaller than 40 years,
   // assuming no system without Unix timestamp
   // runs longer than 40 years continuously without
   // reboot. In worst case this will add/subtract the
   // communication delay between GCS and MAV,
   // it will never alter the timestamp in a safety
   // critical way.
   //
   // Calculation:
   // 40 years
   // 365 days
   // 24 hours
   // 60 minutes
   // 60 seconds
   // 1000 milliseconds
   // 1000 microseconds
#ifndef _MSC_VER
   else if (time < 1261440000000000LLU)
#else
   else if (time < 1261440000000000)
#endif
   {
       //        QLOG_DEBUG() << "GEN time:" << time/1000 + onboardTimeOffset;
       if (onboardTimeOffset == 0)
       {
           onboardTimeOffset = My_GS::groundTimeMilliseconds() - time/1000;
       }
       return time/1000 + onboardTimeOffset;
   }
   else
   {
       // Time is not zero and larger than 40 years -> has to be
       // a Unix epoch timestamp. Do nothing.
       return time/1000;
   }
}

quint64 UAS::getUnixTimeFromMs(quint64 time)
{
   return getUnixTime(time*1000);
}

quint64 UAS::getUnixTime(quint64 time)
{
   quint64 ret = 0;
   if (attitudeStamped)
   {
       ret = lastAttitude;
   }

   if (time == 0)
   {
       ret = My_GS::groundTimeMilliseconds();
   }
   // Check if time is smaller than 40 years,
   // assuming no system without Unix timestamp
   // runs longer than 40 years continuously without
   // reboot. In worst case this will add/subtract the
   // communication delay between GCS and MAV,
   // it will never alter the timestamp in a safety
   // critical way.
   //
   // Calculation:
   // 40 years
   // 365 days
   // 24 hours
   // 60 minutes
   // 60 seconds
   // 1000 milliseconds
   // 1000 microseconds
#ifndef _MSC_VER
   else if (time < 1261440000000000LLU)
#else
   else if (time < 1261440000000000)
#endif
   {
       //        QLOG_DEBUG() << "GEN time:" << time/1000 + onboardTimeOffset;
       if (onboardTimeOffset == 0 || time < (lastNonNullTime - 100))
       {
           lastNonNullTime = time;
           onboardTimeOffset = My_GS::groundTimeMilliseconds() - time/1000;
       }
       if (time > lastNonNullTime) lastNonNullTime = time;

       ret = time/1000 + onboardTimeOffset;
   }
   else
   {
       // Time is not zero and larger than 40 years -> has to be
       // a Unix epoch timestamp. Do nothing.
       ret = time/1000;
   }

   return ret;
}

void UAS::valueChangedRec(const int uasId, const QString& name, const QString& unit, const QVariant& value, const quint64 msec)
{
   emit valueChanged(uasId,name,unit,value,msec);
}


