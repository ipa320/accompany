#include "WidgetHub.h"
#include <QMessageBox>
#include <QDebug>
#include <QObject>

WidgetHub::WidgetHub()
{
}
/*
template <class T>
void WidgetHub::report(T* ui)
{
    connect(ui, SIGNAL(valueChanged(ISensorUI *)),this, SLOT(valueChangedHandler(ISensorUI *)));
}
*/
/*
void WidgetHub::report(ISensorUI* ui)
{
    QObject* x =(QObject*) ui;
    connect(x, SIGNAL(valueChanged(ISensorUI *)),this, SLOT(valueChangedSlot(ISensorUI *)));
}
void WidgetHub::report(ButtonUI* ui)
{
    reportT(ui);
}
void WidgetHub::report(CheckBoxUI* ui)
{
    reportT(ui);
}
*/
void WidgetHub::valueChangedHandler(ISensorUI *ui)
{
//    QMessageBox::information(NULL,ui->getSensorName(),QString::number(ui->getSensorValue()));
    ui->setSensorName(QString::number(ui->getSensorValue()));
}
SensorInfo* WidgetHub::getSensorInfo(int) {
    SensorInfo* si = new SensorInfo();
    si->name = new QString("New Sensor");
    si->type = -1;
    si->location = -1;
    return si;
}

WidgetHub* WidgetHub::factory = new WidgetHub();

