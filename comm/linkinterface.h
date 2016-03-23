#ifndef LINKINTERFACE_H
#define LINKINTERFACE_H

#include <QThread>

#include "gsmavlink.h"
#include "SerialCommunication/serialsettingdialog.h"

class LinkInterface : public QThread
{
    Q_OBJECT

public:
    LinkInterface();
    virtual ~LinkInterface();

    virtual int getId() const = 0;

};

#endif // LINKINTERFACE_H

