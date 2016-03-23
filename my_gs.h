#ifndef MY_GS_H
#define MY_GS_H

#include "comm/gsmavlink.h"
#include <QDateTime>
#include <QColor>
#include <QThread>

#define MAVLINK_HEARTBEAT_DEFAULT_RATE 1

namespace My_GS {
const static int defaultSystemId = 252;
const static int defaultComponentId = MAV_COMP_ID_PRIMARY;

const QColor colorCyan(55, 154, 195);
const QColor colorRed(154, 20, 20);
const QColor colorGreen(20, 200, 20);
const QColor colorYellow(255, 255, 0);
const QColor colorOrange(255, 140, 0);
const QColor colorMagenta(255, 0, 55);
const QColor colorDarkWhite(240, 240, 240);
const QColor colorDarkYellow(180, 180, 0);
const QColor colorBackground("#050508");
const QColor colorBlack(0, 0, 0);

/** @brief Get the current ground time in microseconds */
quint64 groundTimeUsecs();
/** @brief Get the current ground time in milliseconds */
quint64 groundTimeMilliseconds();
/** @brief Get the current ground time in seconds */
qreal groundTimeSeconds();
/** @brief Returns the angle limited to -pi - pi */
float limitAngleToPMPIf(float angle);
/** @brief Returns the angle limited to -pi - pi */
double limitAngleToPMPId(double angle);
}

#endif // MY_GS_H
