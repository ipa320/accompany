#include "isensorui.h"
#include "widgethub.h"
#include <QObject>
#include <QMessageBox>
#include "checkboxui.h"
#include "buttonui.h"
#include <QDebug>
#include "sensorevent.h"
//#include <math.h>

//template <class T>
//static QList<Listener<T> *> Messenger::listeners;

ISensorUI::ISensorUI(bool designerMode) {
    sensorValue = 0; //base value
    setIdCalled = false;
    setDesignerMode(designerMode);
}
void ISensorUI::setSensorId(int id) {
    this->sensorId = id;
    double v = WidgetHub::getFactory()->check(this);
    //double v = WidgetHub::getFactory()->getDbValue(id);
    if(v > -1){
        this->setSensorValue(1);
        this->setSensorEnabled(true);
        if(v != 0) {
            CheckBoxUI *w = dynamic_cast<CheckBoxUI *>(this);
            if(w) w->setChecked(true);
        }
    }
    else {
        this->setSensorEnabled(false);
    }
    if (!setIdCalled) {
        setIdCalled = true;
        return;
    }
    if (getDesignerMode())
    {
        SensorInfo* si = WidgetHub::getFactory()->getSensorInfo(id);
        setSensorName(*si->name);
        setSensorType(si->type);
    }
}
int ISensorUI::getSensorId() {
    return this->sensorId;
}

void ISensorUI::setSensorValue(double value, QString* message) {
    if(this->sensorValue != value) {
        this->sensorValue = value;
        notifyListener(this);
        if (message && getSensorState())
            //direct call, non signal-slot mechanism
            WidgetHub::getFactory()->stateChangedHandler(new SensorEvent(getSensorId(), getSensorValue(),message));
    }
}

double ISensorUI::getSensorValue()
{
//    if(!getSensorState()) return 0;
    return sensorValue;
}

void ISensorUI::setSensorEnabled(bool state) {
    QWidget *w = dynamic_cast<QWidget *>(this);
    if(w) {
        w->setEnabled(state);
    }
    else {
        qDebug() << "Typecasting Error";
    }
}

int ISensorUI::getSensorType()
{
    return sensorType;
}

void ISensorUI::setSensorType(int sensorType)
{
    this->sensorType = sensorType;
    if (getDesignerMode())
    {
        SensorInfo* si = WidgetHub::getFactory()->getSensorInfo(sensorId);
        switch (sensorType)
        {
        case 0:
            setSensorLowMessage(QString("low"));
            setSensorHighMessage(QString("high"));
            break;
        case 1:
        case 2:
        case 4:
            setSensorLowMessage(QString("off"));
            setSensorHighMessage(QString("on"));
            break;
        case 3:
            setSensorLowMessage(QString("unoccupied"));
            setSensorHighMessage(QString("occupied"));
            break;
        //case 4:
            //setSensorLowMessage(QString("inactive"));
            //setSensorHighMessage(*si->rule);
            break;
        case 5:
        default:
            setSensorLowMessage(QString("N/A (Low)"));
            setSensorHighMessage(QString("N/A (High)"));
        }
    }
}

QString ISensorUI::getSensorHighMessage()
{
    return SensorHighMessage;
}

void ISensorUI::setSensorHighMessage(QString SensorHighMessage)
{
    this->SensorHighMessage = SensorHighMessage;
}

QString ISensorUI::getSensorLowMessage()
{
    return SensorLowMessage;
}

void ISensorUI::setSensorLowMessage(QString SensorLowMessage)
{
    this->SensorLowMessage = SensorLowMessage;
}

/* exists in QObject
void ISensorUI::setProperty(const char *name, const QVariant value)
{
    //QInputDialog::getText (NULL, name, "setProperty");
}
QVariant ISensorUI::property(const char *name) const
{
    //QInputDialog::getText (NULL, name, "getProperty");
    QVariant v;
    v.fromValue(987);
    return v;
}
*/

void ISensorUI::setDesignerMode(bool v) {
    designerMode = v;
}
bool ISensorUI::getDesignerMode() {
    return designerMode;
}


