#ifndef CHECKBOXUI_H
#define CHECKBOXUI_H

#include <QCheckBox>
#include <QString>
#include <QWidget>
#include "isensorui.h"
#include "sensorevent.h"

#ifndef plugin
#undef QDESIGNER_WIDGET_EXPORT
#define QDESIGNER_WIDGET_EXPORT
#endif

class QDESIGNER_WIDGET_EXPORT CheckBoxUI : public QCheckBox, public ISensorUI
{
    Q_OBJECT
    Q_PROPERTY(int SensorId READ getSensorId WRITE setSensorId)
    Q_PROPERTY(int SensorType READ getSensorType WRITE setSensorType)
    Q_PROPERTY(QString SensorHighMessage READ getSensorHighMessage WRITE setSensorHighMessage)
    Q_PROPERTY(QString SensorLowMessage READ getSensorLowMessage WRITE setSensorLowMessage)
public:
    explicit CheckBoxUI(QWidget* = 0, bool = false);
    void setSensorName(QString);
    QString getSensorName();
    bool getSensorState();

signals:
    void stateChanged(SensorEvent *);

private slots:
    void clickedHandler();
};

#endif // CHECKBOXUI_H
