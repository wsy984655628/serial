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

    static const double lipoFull;
    static const double LipoEmpty;

    /** @brief Get the unique system id */
    int getUASID() const;
    /** @brief Get the airframe */
    int getAirframe() const
    {
        return airframe;
    }

    Q_PROPERTY(double localX READ getLocalX WRITE setLocalX NOTIFY localXChanged)
    Q_PROPERTY(double localY READ getLocalY WRITE setLocalY NOTIFY localYChanged)
    Q_PROPERTY(double localZ READ getLocalZ WRITE setLocalZ NOTIFY localZChanged)
    Q_PROPERTY(double latitude READ getLatitude WRITE setLatitude NOTIFY latitudeChanged)
    Q_PROPERTY(double longitude READ getLongitude WRITE setLongitude NOTIFY longitudeChanged)
    Q_PROPERTY(double satelliteCount READ getSatelliteCount WRITE setSatelliteCount NOTIFY satelliteCountChanged)
    Q_PROPERTY(bool isLocalPositionKnown READ localPositionKnown)
    Q_PROPERTY(bool isGlobalPositionKnown READ globalPositionKnown)
    Q_PROPERTY(double roll READ getRoll WRITE setRoll NOTIFY rollChanged)
    Q_PROPERTY(double pitch READ getPitch WRITE setPitch NOTIFY pitchChanged)
    Q_PROPERTY(double yaw READ getYaw WRITE setYaw NOTIFY yawChanged)
    Q_PROPERTY(double distToWaypoint READ getDistToWaypoint WRITE setDistToWaypoint NOTIFY distToWaypointChanged)
    Q_PROPERTY(double airSpeed READ getGroundSpeed WRITE setGroundSpeed NOTIFY airSpeedChanged)
    Q_PROPERTY(double groundSpeed READ getGroundSpeed WRITE setGroundSpeed NOTIFY groundSpeedChanged)
    Q_PROPERTY(double bearingToWaypoint READ getBearingToWaypoint WRITE setBearingToWaypoint NOTIFY bearingToWaypointChanged)
    Q_PROPERTY(double altitudeAMSL READ getAltitudeAMSL WRITE setAltitudeAMSL NOTIFY altitudeAMSLChanged)
    Q_PROPERTY(double altitudeRelative READ getAltitudeRelative WRITE setAltitudeRelative NOTIFY altitudeRelativeChanged)



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
    QVector3D getNedPosGlobalOffset() const
    {
        return nedPosGlobalOffset;
    }

    QVector3D getNedAttGlobalOffset() const
    {
        return nedAttGlobalOffset;
    }

protected:
    bool m_heartbeatsEnabled;
    /// LINK ID AND STATUS
    int uasId;                    ///< Unique system ID
    QMap<int, QString> components;///< IDs and names of all detected onboard components
    QList<int> unknownPackets;    ///< Packet IDs which are unknown and have been received
    //MAVLinkProtocol* mavlink;     ///< Reference to the MAVLink instance
    CommStatus commStatus;        ///< Communication status
    double receiveDropRate;        ///< Percentage of packets that were dropped on the MAV's receiving link (from GCS and other MAVs)
    double sendDropRate;           ///< Percentage of packets that were not received from the MAV by the GCS
    quint64 lastHeartbeat;        ///< Time of the last heartbeat message
    QTimer* statusTimeout;        ///< Timer for various status timeouts

    /// BASIC UAS TYPE, NAME AND STATE
    QString name;                 ///< Human-friendly name of the vehicle, e.g. bravo
    unsigned char type;           ///< UAS type (from type enum)
    int airframe;                 ///< The airframe type
    int autopilot;                ///< Type of the Autopilot: -1: None, 0: Generic, 1: PIXHAWK, 2: SLUGS, 3: Ardupilot (up to 15 types), defined in MAV_AUTOPILOT_TYPE ENUM
    bool systemIsArmed;           ///< If the system is armed
    uint8_t base_mode;                 ///< The current mode of the MAV
    uint32_t custom_mode;         ///< The current mode of the MAV
    int status;                   ///< The current status of the MAV
    QString shortModeText;        ///< Short textual mode description
    QString shortStateText;       ///< Short textual state description
    int systemId;                 ///< Currently connected mavlink system id
    int componentId;              ///< Currently connected mavlink component id
    int getSystemId() { return systemId; }
    int getComponentId() { return componentId; }

    /// OUTPUT
    QList<double> actuatorValues;
    QList<QString> actuatorNames;
    QList<double> motorValues;
    QList<QString> motorNames;
    double thrustSum;           ///< Sum of forward/up thrust of all thrust actuators, in Newtons
    double thrustMax;           ///< Maximum forward/up thrust of this vehicle, in Newtons

    // dongfang: This looks like a candidate for being moved off to a separate class.
    /// BATTERY / ENERGY
    BatteryType batteryType;    ///< The battery type
    int cells;                  ///< Number of cells
    double fullVoltage;          ///< Voltage of the fully charged battery (100%)
    double emptyVoltage;         ///< Voltage of the empty battery (0%)
    double startVoltage;         ///< Voltage at system start
    double tickVoltage;          ///< Voltage where 0.1 V ticks are told
    double lastTickVoltageValue; ///< The last voltage where a tick was announced
    double tickLowpassVoltage;   ///< Lowpass-filtered voltage for the tick announcement
    double warnVoltage;          ///< Voltage where QGC will start to warn about low battery
    double warnLevelPercent;     ///< Warning level, in percent
    double currentVoltage;      ///< Voltage currently measured
    double lpVoltage;            ///< Low-pass filtered voltage
    double currentCurrent;      ///< Battery current currently measured
    bool batteryRemainingEstimateEnabled; ///< If the estimate is enabled, QGC will try to estimate the remaining battery life
    double chargeLevel;          ///< Charge level of battery, in percent
    int timeRemaining;          ///< Remaining time calculated based on previous and current
    bool lowBattAlarm;          ///< Switch if battery is low


    /// TIMEKEEPING
    quint64 startTime;            ///< The time the UAS was switched on
    quint64 onboardTimeOffset;

    /// MANUAL CONTROL
    bool manualControl;             ///< status flag, true if roll/pitch/yaw/thrust are controlled manually
    bool overrideRC;                ///< status flag, true if RC overrides are in effect

    /// POSITION
    bool positionLock;          ///< Status if position information is available or not
    bool isLocalPositionKnown;      ///< If the local position has been received for this MAV
    bool isGlobalPositionKnown;     ///< If the global position has been received for this MAV

    double localX;
    double localY;
    double localZ;

    double latitude;            ///< Global latitude as estimated by position estimator
    double longitude;           ///< Global longitude as estimated by position estimator
    double altitudeAMSL;        ///< Global altitude as estimated by position estimator
    double altitudeRelative;    ///< Altitude above home as estimated by position estimator

    int m_satelliteCount;       ///< Number of satellites visible to raw GPS
    double m_gps_hdop;          ///< GPS HDOP
    int m_gps_fix;              ///< GPS FIX type 1, 2 = 2D, 3 = 3D
    bool globalEstimatorActive; ///< Global position estimator present, do not fall back to GPS raw for position
    double latitude_gps;        ///< Global latitude as estimated by raw GPS
    double longitude_gps;       ///< Global longitude as estimated by raw GPS
    double altitude_gps;        ///< Global altitude as estimated by raw GPS
    double speedX;              ///< True speed in X axis
    double speedY;              ///< True speed in Y axis
    double speedZ;              ///< True speed in Z axis

    QVector3D nedPosGlobalOffset;   ///< Offset between the system's NED position measurements and the swarm / global 0/0/0 origin
    QVector3D nedAttGlobalOffset;   ///< Offset between the system's NED position measurements and the swarm / global 0/0/0 origin

    /// WAYPOINT NAVIGATION
    double distToWaypoint;       ///< Distance to next waypoint
    double airSpeed;             ///< Airspeed
    double groundSpeed;          ///< Groundspeed
    double bearingToWaypoint;    ///< Bearing to next waypoint

    /// ATTITUDE
    bool attitudeKnown;             ///< True if attitude was received, false else
    bool attitudeStamped;           ///< Should arriving data be timestamped with the last attitude? This helps with broken system time clocks on the MAV
    quint64 lastAttitude;           ///< Timestamp of last attitude measurement
    double roll;
    double pitch;
    double yaw;

    // dongfang: This looks like a candidate for being moved off to a separate class.
    /// IMAGING
    int imageSize;              ///< Image size being transmitted (bytes)
    int imagePackets;           ///< Number of data packets being sent for this image
    int imagePacketsArrived;    ///< Number of data packets recieved
    int imagePayload;           ///< Payload size per transmitted packet (bytes). Standard is 254, and decreases when image resolution increases.
    int imageQuality;           ///< Quality of the transmitted image (percentage)
    int imageType;              ///< Type of the transmitted image (BMP, PNG, JPEG, RAW 8 bit, RAW 32 bit)
    int imageWidth;             ///< Width of the image stream
    int imageHeight;            ///< Width of the image stream
    QByteArray imageRecBuffer;  ///< Buffer for the incoming bytestream
    QImage image;               ///< Image data of last completely transmitted image
    quint64 imageStart;
    bool blockHomePositionChanges;   ///< Block changes to the home position
    bool receivedMode;          ///< True if mode was retrieved from current conenction to UAS

    /// PARAMETERS
    QMap<int, QMap<QString, QVariant>* > parameters; ///< All parameters
    bool paramsOnceRequested;       ///< If the parameter list has been read at least once

public:
    void setHeartbeatEnabled(bool enabled) { m_heartbeatsEnabled = enabled; }
    /** @brief Estimate how much flight time is remaining */
    int calculateTimeRemaining();
    /** @brief Get the current charge level */
    double getChargeLevel();

    /** @brief Check if vehicle is in autonomous mode */
    bool isAuto();
    /** @brief Check if vehicle is armed */
    bool isArmed() const { return systemIsArmed; }
    /** @brief Check if vehicle is in HIL mode */
    bool isHilEnabled() const { return hilEnabled; }


    /**
     * @brief Returns true for systems that can reverse. If the system has no control over position, it returns false as
     * @return If the specified vehicle type can
     */
    bool systemCanReverse() const
    {
        switch(type)
        {
        case MAV_TYPE_GENERIC:
        case MAV_TYPE_FIXED_WING:
        case MAV_TYPE_ROCKET:
        case MAV_TYPE_FLAPPING_WING:

        // System types that don't have movement
        case MAV_TYPE_ANTENNA_TRACKER:
        case MAV_TYPE_GCS:
        case MAV_TYPE_FREE_BALLOON:
        default:
            return false;
        case MAV_TYPE_QUADROTOR:
        case MAV_TYPE_COAXIAL:
        case MAV_TYPE_HELICOPTER:
        case MAV_TYPE_AIRSHIP:
        case MAV_TYPE_GROUND_ROVER:
        case MAV_TYPE_SURFACE_BOAT:
        case MAV_TYPE_SUBMARINE:
        case MAV_TYPE_HEXAROTOR:
        case MAV_TYPE_OCTOROTOR:
        case MAV_TYPE_TRICOPTER:
            return true;
        }
    }

    QString getSystemTypeName()
    {
        switch(type)
        {
        case MAV_TYPE_GENERIC:
            return "GENERIC";
            break;
        case MAV_TYPE_FIXED_WING:
            return "FIXED_WING";
            break;
        case MAV_TYPE_QUADROTOR:
            return "QUADROTOR";
            break;
        case MAV_TYPE_COAXIAL:
            return "COAXIAL";
            break;
        case MAV_TYPE_HELICOPTER:
            return "HELICOPTER";
            break;
        case MAV_TYPE_ANTENNA_TRACKER:
            return "ANTENNA_TRACKER";
            break;
        case MAV_TYPE_GCS:
            return "GCS";
            break;
        case MAV_TYPE_AIRSHIP:
            return "AIRSHIP";
            break;
        case MAV_TYPE_FREE_BALLOON:
            return "FREE_BALLOON";
            break;
        case MAV_TYPE_ROCKET:
            return "ROCKET";
            break;
        case MAV_TYPE_GROUND_ROVER:
            return "GROUND_ROVER";
            break;
        case MAV_TYPE_SURFACE_BOAT:
            return "BOAT";
            break;
        case MAV_TYPE_SUBMARINE:
            return "SUBMARINE";
            break;
        case MAV_TYPE_HEXAROTOR:
            return "HEXAROTOR";
            break;
        case MAV_TYPE_OCTOROTOR:
            return "OCTOROTOR";
            break;
        case MAV_TYPE_TRICOPTER:
            return "TRICOPTER";
            break;
        case MAV_TYPE_FLAPPING_WING:
            return "FLAPPING_WING";
            break;
        default:
            return "";
            break;
        }
    }

    QString getAutopilotTypeName()
    {
        switch (autopilot)
        {
        case MAV_AUTOPILOT_GENERIC:
            return "GENERIC";
            break;
        case MAV_AUTOPILOT_PIXHAWK:
            return "PIXHAWK";
            break;
        case MAV_AUTOPILOT_SLUGS:
            return "SLUGS";
            break;
        case MAV_AUTOPILOT_ARDUPILOTMEGA:
            return "ARDUPILOTMEGA";
            break;
        case MAV_AUTOPILOT_OPENPILOT:
            return "OPENPILOT";
            break;
        case MAV_AUTOPILOT_GENERIC_WAYPOINTS_ONLY:
            return "GENERIC_WAYPOINTS_ONLY";
            break;
        case MAV_AUTOPILOT_GENERIC_WAYPOINTS_AND_SIMPLE_NAVIGATION_ONLY:
            return "GENERIC_MISSION_NAVIGATION_ONLY";
            break;
        case MAV_AUTOPILOT_GENERIC_MISSION_FULL:
            return "GENERIC_MISSION_FULL";
            break;
        case MAV_AUTOPILOT_INVALID:
            return "NO AP";
            break;
        case MAV_AUTOPILOT_PPZ:
            return "PPZ";
            break;
        case MAV_AUTOPILOT_UDB:
            return "UDB";
            break;
        case MAV_AUTOPILOT_FP:
            return "FP";
            break;
        case MAV_AUTOPILOT_PX4:
            return "PX4";
            break;
        default:
            return "";
            break;
        }
    }



public slots:
    void valueChangedRec(const int uasId, const QString& name, const QString& unit, const QVariant& value, const quint64 msec);
    /** @brief Set the autopilot type */
    void setAutopilotType(int apType)
    {
        autopilot = apType;
        emit systemSpecsChanged(uasId);
    }
    /** @brief Set the specific airframe type */

    /** @brief Receive a message from one of the communication links. */
    void receiveMessage( mavlink_message_t message);

signals:
    void loadChanged(UASInterface* uas, double load);
    /** @brief Propagate a heartbeat received from the system */
    //void heartbeat(UASInterface* uas); // Defined in UASInterface already

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
    bool connectionLost; ///< Flag indicates a timed out connection
    quint64 connectionLossTime; ///< Time the connection was interrupted
    quint64 lastVoltageWarning; ///< Time at which the last voltage warning occured
    quint64 lastNonNullTime;    ///< The last timestamp from the MAV that was not null
    unsigned int onboardTimeOffsetInvalidCount;     ///< Count when the offboard time offset estimation seemed wrong
    bool hilEnabled;            ///< Set to true if HIL mode is enabled from GCS (UAS might be in HIL even if this flag is not set, this defines the GCS HIL setting)
    bool sensorHil;             ///< True if sensor HIL is enabled
    quint64 lastSendTimeGPS;     ///< Last HIL GPS message sent
    quint64 lastSendTimeSensors;

    QList< QPair<int, QString> >  paramRequestQueue;

    QTimer m_parameterSendTimer;


protected slots:
    void requestNextParamFromQueue();

};

#endif // UAS_H
