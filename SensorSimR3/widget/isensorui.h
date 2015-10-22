#ifndef ISENSORUI_H
#define ISENSORUI_H

#include <QString>
#include <QObject>
#include <QVariant>
#include "db/sensorinfo.h"
//#include "widgetmanager.h" //pre-circular error if declared
#include <QList>
#include <QMessageBox>
template <class T>
class Listener
{
public:
    virtual void notificationHandler(T* t) = 0;
};
template <class T>
class Messenger
{
public:
    void addListener(Listener<T>* l)
    {
        listeners.append(l);
    }
    void removeListener(Listener<T>* l)
    {
        listeners.removeAll(l);
    }
    void notifyListener(T* t)
    {
        foreach (Listener<T> *l, listeners)
        {
            l->notificationHandler(t);
        }
    }
private:
    QList<Listener<T> *> listeners;
};

class ISensorUI: public Messenger<ISensorUI> {// : private QObject{
//    Q_OBJECT
public:
    ISensorUI(bool);

    virtual void setSensorId(int id);
    virtual int getSensorId();
    virtual void setSensorName(QString) = 0;
    virtual QString getSensorName() = 0;
    virtual void setSensorValue(double, QString* msg = NULL);
    virtual double getSensorValue();
    virtual void setSensorState(bool) {}
    virtual bool getSensorState() = 0;
    virtual void setSensorEnabled(bool state);
    virtual int getSensorType();
    virtual void setSensorType(int);
    virtual QString getSensorHighMessage();
    virtual void setSensorHighMessage(QString);
    virtual QString getSensorLowMessage();
    virtual void setSensorLowMessage(QString);


    /* exists in QObject
    virtual void setProperty(const char *name, const QVariant value);
    virtual QVariant property(const char *name) const;
    */

    void setDesignerMode(bool v);
    bool getDesignerMode();

private:
    int sensorId;
    bool designerMode;
    bool setIdCalled;
    int sensorType;
    QString SensorLowMessage;
    QString SensorHighMessage;

protected:
    int sensorValue;
};

#endif // ISENSORUI_H
