#include "sensorevent.h"
/*
SensorEvent::SensorEvent()
{
}
*/
SensorEvent::SensorEvent(int id, double value, QString* message)
{
    this->id = id;
    this->value = value;
    this->message = message;
}
