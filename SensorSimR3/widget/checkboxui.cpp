#include "checkboxui.h"
#include "widgethub.h"
#include <QCheckBox>
#include <QString>
#include "sensorevent.h"

CheckBoxUI::CheckBoxUI(QWidget *parent, bool designerMode) :  QCheckBox(parent), ISensorUI(designerMode)
{
//    this->sensorValue = 1;
    WidgetHub::getFactory()->report(this);
    QObject::connect(this, SIGNAL(clicked()), this, SLOT(clickedHandler()));
}

void CheckBoxUI::setSensorName(QString name) {
    this->setText(name);

}
QString CheckBoxUI::getSensorName()
{
    return this->text();
}
bool CheckBoxUI::getSensorState()
{
    if(this->checkState()) return true;
    else return false;
}

void CheckBoxUI::clickedHandler()
{
    if(getSensorState())
    {
        emit stateChanged(new SensorEvent(getSensorId(),getSensorValue(),&getSensorHighMessage()));
    }
    else
    {
        emit stateChanged(new SensorEvent(getSensorId(),0,&getSensorLowMessage()));
    }
}
