#include "uasmanager.h"
#include <QApplication>
#include "uas.h"
#include "uasinterface.h"

UASManager* UASManager::instance()
{
    static UASManager* _instance = 0;

    if(_instance == 0)
    {
        _instance = new UASManager();
        _instance->setParent(qApp);
    }
    return _instance;
}

UASManager::UASManager() :
        activeUAS(NULL),
        homeLat(32.835354),
        homeLon(-117.162774),
        homeAlt(25.0),
        homeFrame(MAV_FRAME_GLOBAL)
{
}

UASManager::~UASManager()
{
    // Delete all systems
    foreach (UASInterface* mav, systems) {
        delete mav;
    }
}

UASInterface* UASManager::getUASForId(int id)
{
    UASInterface* system = NULL;
    foreach (UASInterface* sys, systems) {
        if(sys->getUASID() == id)
        {
            system = sys;
        }
    }

    return system;
}

void UASManager::addUAS(UASInterface *uas)
{
   if (!systems.contains(uas))
   {
       systems.append(uas);
   }
}

