#include "buttonui.h"
#include "widgethub.h"
#include <QPushButton>
#include <QString>
#include <QDebug>
#include <QMessageBox>

ButtonUI::ButtonUI(QWidget *parent, bool designerMode) :  QPushButton(parent), ISensorUI(designerMode)
{
    //this->sensorValue = 1;
    WidgetHub::getFactory()->report(this);
    connect(this, SIGNAL(pressed()), this, SLOT(buttonPressedHandler()));
    connect(this, SIGNAL(released()), this, SLOT(buttonReleasedHandler()));
}

void ButtonUI::setSensorName(QString name) {
    this->setText(name);

}
QString ButtonUI::getSensorName()
{
    return this->text();
}
bool ButtonUI::getSensorState()
{
    return tempVal;
}

void ButtonUI::buttonPressedHandler()
{
    tempVal = true;
    emit stateChanged(new SensorEvent(getSensorId(),getSensorValue(), &getSensorHighMessage()));
}
void ButtonUI::buttonReleasedHandler()
{
    tempVal = false;
    emit stateChanged(new SensorEvent(getSensorId(),0, &getSensorLowMessage()));
}
