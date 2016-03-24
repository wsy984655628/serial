#ifndef UAS_H
#define UAS_H

#include "uas/uasinterface.h"
#include "comm/mavlinkprotocol.h"
#include <QVector3D>
#include <QTimer>
#include "comm/gsmavlink.h"


class UAS : public UASInterface
{
public:

    UAS(MAVLINKProtocol* protocol, int id = 0);
    ~UAS();

    /** @brief Get the unique system id */
    int getUASID() const;
    /** @brief Get the airframe */

    Q_PROPERTY(double roll READ getRoll WRITE setRoll NOTIFY rollChanged)
    Q_PROPERTY(double pitch READ getPitch WRITE setPitch NOTIFY pitchChanged)
    Q_PROPERTY(double yaw READ getYaw WRITE setYaw NOTIFY yawChanged)

    void setRoll(double val)
    {
        roll = val;
//        emit rollChanged(val,"roll");
    }

    double getRoll() const
    {
        return roll;
    }

    void setPitch(double val)
    {
        pitch = val;
//        emit pitchChanged(val,"pitch");
    }

    double getPitch() const
    {
        return pitch;
    }

    void setYaw(double val)
    {
        yaw = val;
//        emit yawChanged(val,"yaw");
    }

    double getYaw() const
    {
        return yaw;
    }


protected:
    /// LINK ID AND STATUS
    int uasId;                    ///< Unique system ID
    QMap<int, QString> components;///< IDs and names of all detected onboard components
    QList<int> unknownPackets;    ///< Packet IDs which are unknown and have been received
    //MAVLinkProtocol* mavlink;     ///< Reference to the MAVLink instance

    /// BASIC UAS TYPE, NAME AND STATE
    QString name;                 ///< Human-friendly name of the vehicle, e.g. bravo
    unsigned char type;           ///< UAS type (from type enum)
    int autopilot;                ///< Type of the Autopilot: -1: None, 0: Generic, 1: PIXHAWK, 2: SLUGS, 3: Ardupilot (up to 15 types), defined in MAV_AUTOPILOT_TYPE ENUM
    uint8_t base_mode;                 ///< The current mode of the MAV
    uint32_t custom_mode;         ///< The current mode of the MAV
    int status;                   ///< The current status of the MAV
    int systemId;                 ///< Currently connected mavlink system id
    int componentId;              ///< Currently connected mavlink component id
    int getSystemId() { return systemId; }
    int getComponentId() { return componentId; }

    // dongfang: This looks like a candidate for being moved off to a separate class.

    /// TIMEKEEPING
    quint64 onboardTimeOffset;

    /// ATTITUDE
    bool attitudeStamped;           ///< Should arriving data be timestamped with the last attitude? This helps with broken system time clocks on the MAV
    quint64 lastAttitude;           ///< Timestamp of last attitude measurement
    double roll;
    double pitch;
    double yaw;

public slots:
    void valueChangedRec(const int uasId, const QString& name, const QString& unit, const QVariant& value, const quint64 msec);
    /** @brief Set the autopilot type */
    void setAutopilotType(int apType)
    {
        autopilot = apType;
    }
    /** @brief Set the specific airframe type */

    /** @brief Receive a message from one of the communication links. */
    void receiveMessage( mavlink_message_t message);

signals:
    void rollChanged(double val,QString name);
    void pitchChanged(double val,QString name);
    void yawChanged(double val,QString name);
protected:
    /** @brief Get the UNIX timestamp in milliseconds, enter microseconds */
    quint64 getUnixTime(quint64 time=0);
    /** @brief Get the UNIX timestamp in milliseconds, enter milliseconds */
    quint64 getUnixTimeFromMs(quint64 time);
    /** @brief Get the UNIX timestamp in milliseconds, ignore attitudeStamped mode */
    quint64 getUnixReferenceTime(quint64 time);

    int componentID[256];
    bool componentMulti[256];
    quint64 lastNonNullTime;    ///< The last timestamp from the MAV that was not null
};

#endif // UAS_H
