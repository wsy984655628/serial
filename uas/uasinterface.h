#ifndef UASINTERFACE
#define UASINTERFACE

#include <QObject>
#include <QList>

#include "comm/gsmavlink.h"

enum BatteryType
{
    NICD = 0,
    NIMH = 1,
    LIION = 2,
    LIPOLY = 3,
    LIFE = 4,
    AGZN = 5
}; ///< The type of battery used

class UASInterface : public QObject
{
    Q_OBJECT
public:
    virtual ~UASInterface() {}

    virtual void setHeartbeatEnabled(bool enabled) = 0;

    virtual int getUASID() const = 0;
    virtual double getRoll() const = 0;
    virtual double getPitch() const = 0;
    virtual double getYaw() const = 0;

    virtual bool isArmed() const = 0;

    virtual int getAirframe() const = 0;

    enum CommStatus {
        /** Unit is disconnected, no failure state reached so far **/
        COMM_DISCONNECTED = 0,
        /** The communication is established **/
        COMM_CONNECTING = 1,
        /** The communication link is up **/
        COMM_CONNECTED = 2,
        /** The connection is closed **/
        COMM_DISCONNECTING = 3,
        COMM_FAIL = 4,
        COMM_TIMEDOUT = 5///< Communication link failed
    };
    enum Airframe {
        QGC_AIRFRAME_GENERIC = 0,
        QGC_AIRFRAME_EASYSTAR,
        QGC_AIRFRAME_TWINSTAR,
        QGC_AIRFRAME_MERLIN,
        QGC_AIRFRAME_CHEETAH,
        QGC_AIRFRAME_MIKROKOPTER,
        QGC_AIRFRAME_REAPER,
        QGC_AIRFRAME_PREDATOR,
        QGC_AIRFRAME_COAXIAL,
        QGC_AIRFRAME_PTERYX,
        QGC_AIRFRAME_TRICOPTER,
        QGC_AIRFRAME_HEXCOPTER,
        QGC_AIRFRAME_X8,
        QGC_AIRFRAME_VIPER_2_0,
        QGC_AIRFRAME_CAMFLYER_Q,
        QGC_AIRFRAME_HELICOPTER,
        QGC_AIRFRAME_END_OF_ENUM
    };

    virtual bool systemCanReverse() const = 0;

    virtual QString getSystemTypeName() = 0;

    virtual QString getAutopilotTypeName() = 0;
    virtual void setAutopilotType(int apType) = 0;

public slots:
    virtual void valueChangedRec(const int uasId, const QString& name, const QString& unit, const QVariant& value, const quint64 msec)=0;

    /** @brief Receive a message from one of the communication links. */
    virtual void receiveMessage(mavlink_message_t message) = 0;

    virtual int getSystemId()=0;
    virtual int getComponentId()=0;

signals:
    /** @brief The robot state has changed */
    void statusChanged(int stateFlag);
    /** @brief A new component was detected or created */
    void componentCreated(int uas, int component, const QString& name);
    /** @brief The robot state has changed
     *
     * @param uas this robot
     * @param status short description of status, e.g. "connected"
     * @param description longer textual description. Should be however limited to a short text, e.g. 200 chars.
     */
    void statusChanged(UASInterface* uas, QString status, QString description);
    /** @brief System has been removed / disconnected / shutdown cleanly, remove */
    void systemRemoved(UASInterface* uas);
    void systemRemoved();
    /**
     * @brief Received a plain text message from the robot
     * This signal should NOT be used for standard communication, but rather for VERY IMPORTANT
     * messages like critical errors.
     *
     * @param uasid ID of the sending system
     * @param compid ID of the sending component
     * @param text the status text
     * @param severity The severity of the message, 0 for plain debug messages, 10 for very critical messages
     */

    void poiFound(UASInterface* uas, int type, int colorIndex, QString message, float x, float y, float z);
    void poiConnectionFound(UASInterface* uas, int type, int colorIndex, QString message, float x1, float y1, float z1, float x2, float y2, float z2);

    /** @brief A text message from the system has been received */
    void textMessageReceived(int uasid, int componentid, int severity, QString text);

    void navModeChanged(int uasid, int mode, const QString& text);

    /** @brief System is now armed */
    void armed();
    /** @brief System is now disarmed */
    void disarmed();
    /** @brief Arming mode changed */
    void armingChanged(bool armed);

    /**
     * @brief Update the error count of a device
     *
     * The error count indicates how many errors occured during the use of a device.
     * Usually a random error from time to time is acceptable, e.g. through electromagnetic
     * interferences on device lines like I2C and SPI. A constantly and rapidly increasing
     * error count however can help to identify broken cables or misbehaving drivers.
     *
     * @param uasid System ID
     * @param component Name of the component, e.g. "IMU"
     * @param device Name of the device, e.g. "SPI0" or "I2C1"
     * @param count Errors occured since system startup
     */
    void errCountChanged(int uasid, QString component, QString device, int count);

    /**
     * @brief Drop rate of communication link updated
     *
     * @param systemId id of the air system
     * @param receiveDrop drop rate of packets this MAV receives (sent from GCS or other MAVs)
     */
    void dropRateChanged(int systemId,  float receiveDrop);
    /** @brief Robot mode has changed */
    void modeChanged(int sysId, QString status, QString description);
    /** @brief Robot armed state has changed */
    void armingChanged(int sysId, QString armingState);
    /** @brief A command has been issued **/
    void commandSent(int command);
    /** @brief The connection status has changed **/
    void connectionChanged(CommStatus connectionFlag);
    /** @brief The robot is connecting **/
    void connecting();
    /** @brief The robot is connected **/
    void connected();
    /** @brief The robot is disconnected **/
    void disconnected();
    /** @brief The robot is active **/
    void activated();
    /** @brief The robot is inactive **/
    void deactivated();
    /** @brief The robot is manually controlled **/
    void manualControl();

    /** @brief A value of the robot has changed.
      *
      * Typically this is used to send lowlevel information like the battery voltage to the plotting facilities of
      * the groundstation. The data here should be converted to human-readable values before being passed, so ideally
      * SI units.
      *
      * @param uasId ID of this system
      * @param name name of the value, e.g. "battery voltage"
      * @param unit The units this variable is in as an abbreviation. For system-dependent (such as raw ADC values) use "raw", for bitfields use "bits", for true/false or on/off use "bool", for unitless values use "-".
      * @param value the value that changed
      * @param msec the timestamp of the message, in milliseconds
      */
    void valueChanged(const int uasid, const QString& name, const QString& unit, const QVariant &value,const quint64 msecs);

    void voltageChanged(int uasId, double voltage);
    void waypointUpdated(int uasId, int id, double x, double y, double z, double yaw, bool autocontinue, bool active);
    void waypointSelected(int uasId, int id);
    void waypointReached(UASInterface* uas, int id);
    void autoModeChanged(bool autoMode);
//    void parameterChanged(int uas, int component, QString parameterName, QVariant value);
//    void parameterChanged(int uas, int component, int parameterCount, int parameterId, QString parameterName, QVariant value);
    void patternDetected(int uasId, QString patternPath, float confidence, bool detected);
    void letterDetected(int uasId, QString letter, float confidence, bool detected);

    /**
     * @brief The battery status has been updated
     *
     * @param uas sending system
     * @param voltage battery voltage
     * @param percent remaining capacity in percent
     * @param seconds estimated remaining flight time in seconds
     */
    void batteryChanged(UASInterface* uas, double voltage, double current, double percent, int seconds);
    void statusChanged(UASInterface* uas, QString status);
    void actuatorChanged(UASInterface*, int actId, double value);
    void thrustChanged(UASInterface*, double thrust);
    void heartbeat(UASInterface* uas);
    void attitudeChanged(UASInterface*, double roll, double pitch, double yaw, quint64 usec);
    void attitudeChanged(UASInterface*, int component, double roll, double pitch, double yaw, quint64 usec);
    void attitudeRotationRatesChanged(int uas, double rollrate, double pitchrate, double yawrate, quint64 usec);
    void attitudeThrustSetPointChanged(UASInterface*, double rollDesired, double pitchDesired, double yawDesired, double thrustDesired, quint64 usec);

    /** @brief The MAV set a new setpoint in the local (not body) NED X, Y, Z frame */
    void positionSetPointsChanged(int uasid, float xDesired, float yDesired, float zDesired, float yawDesired, quint64 usec);
    /** @brief A user (or an autonomous mission or obstacle avoidance planner) requested to set a new setpoint */
    void userPositionSetPointsChanged(int uasid, float xDesired, float yDesired, float zDesired, float yawDesired);
    void localPositionChanged(UASInterface*, double x, double y, double z, quint64 usec);
    void localPositionChanged(UASInterface*, int component, double x, double y, double z, quint64 usec);
    void globalPositionChanged(UASInterface*, double lat, double lon, double alt, quint64 usec);
    void altitudeChanged(UASInterface*, double altitudeAMSL, double altitudeRelative, double climbRate, quint64 usec);
    /** @brief Update the status of one satellite used for localization */
    void gpsSatelliteStatusChanged(int uasid, int satid, float azimuth, float direction, float snr, bool used);

    // The horizontal speed (a scalar)
    void speedChanged(UASInterface* uas, double groundSpeed, double airSpeed, quint64 usec);
    // Consider adding a MAV_FRAME parameter to this; could help specifying what the 3 scalars are.
    void velocityChanged_NED(UASInterface*, double vx, double vy, double vz, quint64 usec);

    void navigationControllerErrorsChanged(UASInterface*, double altitudeError, double speedError, double xtrackError);

    void imageStarted(int imgid, int width, int height, int depth, int channels);
    void imageDataReceived(int imgid, const unsigned char* imageData, int length, int startIndex);
    /** @brief Emit the new system type */
    void systemTypeSet(UASInterface* uas, unsigned int type);

    /** @brief Attitude control enabled/disabled */
    void attitudeControlEnabled(bool enabled);
    /** @brief Position 2D control enabled/disabled */
    void positionXYControlEnabled(bool enabled);
    /** @brief Altitude control enabled/disabled */
    void positionZControlEnabled(bool enabled);
    /** @brief Heading control enabled/disabled */
    void positionYawControlEnabled(bool enabled);
    /** @brief Optical flow status changed */
    void opticalFlowStatusChanged(bool supported, bool enabled, bool ok);
    /** @brief Vision based localization status changed */
    void visionLocalizationStatusChanged(bool supported, bool enabled, bool ok);
    /** @brief Infrared / Ultrasound status changed */
    void distanceSensorStatusChanged(bool supported, bool enabled, bool ok);
    /** @brief Gyroscope status changed */
    void gyroStatusChanged(bool supported, bool enabled, bool ok);
    /** @brief Accelerometer status changed */
    void accelStatusChanged(bool supported, bool enabled, bool ok);
    /** @brief Magnetometer status changed */
    void magSensorStatusChanged(bool supported, bool enabled, bool ok);
    /** @brief Barometer status changed */
    void baroStatusChanged(bool supported, bool enabled, bool ok);
    /** @brief Differential pressure / airspeed status changed */
    void airspeedStatusChanged(bool supported, bool enabled, bool ok);
    /** @brief Actuator status changed */
    void actuatorStatusChanged(bool supported, bool enabled, bool ok);
    /** @brief Laser scanner status changed */
    void laserStatusChanged(bool supported, bool enabled, bool ok);
    /** @brief Vicon / Leica Geotracker status changed */
    void groundTruthSensorStatusChanged(bool supported, bool enabled, bool ok);


    /** @brief Value of a remote control channel (raw) */
    void remoteControlChannelRawChanged(int channelId, float raw);
    /** @brief Value of a remote control channel (scaled)*/
    void remoteControlChannelScaledChanged(int channelId, float normalized);
    /** @brief Remote control RSSI changed */
    void remoteControlRSSIChanged(float rssi);
    /** @brief Radio Calibration Data has been received from the MAV*/
//    void radioCalibrationReceived(const QPointer<RadioCalibrationData>&);

    /**
     * @brief Localization quality changed
     * @param fix 0: lost, 1: 2D local position hold, 2: 2D localization, 3: 3D localization
     */
    void localizationChanged(UASInterface* uas, int fix);
    /**
     * @brief GPS localization quality changed
     * @param fix 0: lost, 1: at least one satellite, but no GPS fix, 2: 2D localization, 3: 3D localization
     */
    void gpsLocalizationChanged(UASInterface* uas, int fix);
    /**
     * @brief Vision localization quality changed
     * @param fix 0: lost, 1: 2D local position hold, 2: 2D localization, 3: 3D localization
     */
    void visionLocalizationChanged(UASInterface* uas, int fix);
    /**
     * @brief IR/U localization quality changed
     * @param fix 0: No IR/Ultrasound sensor, N > 0: Found N active sensors
     */
    void irUltraSoundLocalizationChanged(UASInterface* uas, int fix);

    // ERROR AND STATUS SIGNALS
    /** @brief Heartbeat timed out or was regained */
    void heartbeatTimeout(bool timeout, unsigned int ms);
    /** @brief Name of system changed */
    void nameChanged(QString newName);
    /** @brief System has been selected as focused system */
    void systemSelected(bool selected);
    /** @brief Core specifications have changed */
    void systemSpecsChanged(int uasId);

    /** @brief Object detected */
    void objectDetected(unsigned int time, int id, int type, const QString& name, int quality, float bearing, float distance);


    // HOME POSITION / ORIGIN CHANGES
    void homePositionChanged(int uas, double lat, double lon, double alt);

    /** @brief RAW IMU message used for calculating offsets etc */
    void rawImuMessageUpdate(UASInterface *uas, mavlink_raw_imu_t rawImu);
    /** @brief RAW IMU message used for calculating offsets etc */
    void scaledImuMessageUpdate(UASInterface *uas, mavlink_scaled_imu_t scaledImu);
    /** @brief RAW IMU message used for calculating offsets etc */
    void scaledImu2MessageUpdate(UASInterface *uas, mavlink_scaled_imu2_t scaledImu2);
    /** @brief Sensor Offset update message*/
    void sensorOffsetsMessageUpdate(UASInterface *uas, mavlink_sensor_offsets_t sensorOffsets);
    /** @brief Radio Status update message*/
    void radioMessageUpdate(UASInterface *uas, mavlink_radio_t radioMessage);
    /** @brief Compass Mot Status update message*/
    void compassMotCalibration(mavlink_compassmot_status_t* compassmot_status);
    /** @brief Range Finder update message*/
    void rangeFinderUpdate(UASInterface *uas, double distance, double voltage);

    // Log Download Signals
    void logEntry(int uasId, uint32_t time_utc, uint32_t size, uint16_t id, uint16_t num_logs, uint16_t last_log_num);
    void logData(uint32_t uasId, uint32_t ofs, uint16_t id, uint8_t count, const char* data);

    void protocolStatusMessage(const QString& title, const QString& message);
    //void valueChanged(const int uasId, const QString& name, const QString& unit, const QVariant& value, const quint64 msec);
    //void textMessageReceived(int uasid, int componentid, int severity, const QString& text);
    void receiveLossChanged(int id,float value);

    void mavlinkMessageRecieved(mavlink_message_t message);

protected:
    static const unsigned int timeoutIntervalHeartbeat = 10000 * 1000; ///< Heartbeat timeout is 10 seconds

};

#endif // UASINTERFACE

