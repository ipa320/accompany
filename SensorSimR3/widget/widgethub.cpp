#include "widgethub.h"
#include <QMessageBox>
#include <QDebug>
#include <QObject>

WidgetHub* WidgetHub::factory = new WidgetHub();

WidgetHub::WidgetHub() {
    counter = 0;
}

WidgetHub* WidgetHub::getFactory() {
    return factory;
}
DbSvc* WidgetHub::getDbFactory() {
    return DbSvc::getFactory();
}

SensorInfo* WidgetHub::getSensorInfo(int sensorId) {
    return DbSvc::getFactory()->getSensorInfo(sensorId);
}

void WidgetHub::stateChangedHandler(SensorEvent *ev) {
    DbSvc::getFactory()->updateSensorLog(ev->id,ev->value, ev->message);
}

double WidgetHub::getDbValue(int sensorId) {

    return DbSvc::getFactory()->getDbValue(sensorId);
}
void WidgetHub::reportUnregisteredSensor()
{
    QHash<int, int>::iterator i;
    for (i = sensorList->begin(); i != sensorList->end(); ++i)
    {
        qDebug() << "WidgetHub: Unregistered sensor (Id/LocationId): " << i.key() << i.value();
    }
}
