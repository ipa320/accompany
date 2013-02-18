#ifndef BUTTONUI_H
#define BUTTONUI_H

#include <QPushButton>
#include <QString>
#include <QWidget>
#include "isensorui.h"
#include "sensorevent.h"

#ifndef plugin
#undef QDESIGNER_WIDGET_EXPORT
#define QDESIGNER_WIDGET_EXPORT
#endif

class QDESIGNER_WIDGET_EXPORT ButtonUI : public QPushButton, public ISensorUI
{
    Q_OBJECT
    Q_PROPERTY(int SensorId READ getSensorId WRITE setSensorId)
    Q_PROPERTY(QString SensorHighMessage READ getSensorHighMessage WRITE setSensorHighMessage)
    Q_PROPERTY(QString SensorLowMessage READ getSensorLowMessage WRITE setSensorLowMessage)

public:
    explicit ButtonUI(QWidget* = 0, bool = false);
    void setSensorName(QString);
    QString getSensorName();
    bool getSensorState();

signals:
    void stateChanged(SensorEvent *);

private slots:
    void buttonPressedHandler();
    void buttonReleasedHandler();
private:
    bool tempVal;
};

#endif // ButtonUI_H
