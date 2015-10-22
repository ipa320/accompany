#ifndef UIEVENT_H
#define UIEVENT_H
#include <QString>

class SensorEvent
{
public:
    //SensorEvent();
    SensorEvent(int id, double  value, QString* message);
    int id;
    double  value;
    QString* message;
};

#endif // UIEVENT_H
