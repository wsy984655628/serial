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

public slots:
    void addUAS(UASInterface* UAS);
};

#endif // UASMANAGER_H
