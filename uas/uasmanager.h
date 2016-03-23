#ifndef UASMANAGER_H
#define UASMANAGER_H

#include <QObject>
#include <QThread>
#include <QList>
#include <QMutex>
#include <QVector3D>
#include <QQuaternion>

#include "uas/uasinterface.h"


class UASManager : public QObject
{
    Q_OBJECT
public:
    static UASManager* instance();
    ~UASManager();

    UASInterface* getUASForId(int id);

protected:
    UASManager();
    QList<UASInterface*> systems;
    UASInterface* activeUAS;
    QMutex activeUASMutex;
    double homeLat;
    double homeLon;
    double homeAlt;
    int homeFrame;


public slots:
    void addUAS(UASInterface* UAS);
signals:
    /** @brief The UAS currently under main operator control changed */
    void activeUASStatusChanged(UASInterface* UAS, bool active);
    /** @brief The UAS currently under main operator control changed */
    void activeUASStatusChanged(int systemId, bool active);
    /** @brief The UAS currently under main operator control changed */
    void activeUASSet(UASInterface* UAS);
    /** @brief The UAS currently under main operator control changed */
    void activeUASSet(int systemId);
    /** @brief The UAS currently under main operator control changed */
    void activeUASSetListIndex(int listIndex);
    /** A system was deleted */
    void UASDeleted(UASInterface* UAS);
    void UASCreated(UASInterface* UAS);
};

#endif // UASMANAGER_H
