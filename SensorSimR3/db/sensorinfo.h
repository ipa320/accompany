#ifndef SENSORINFO_H
#define SENSORINFO_H
#include <QString>

class SensorInfo{
public:
    SensorInfo(){}
    QString* name;
    int type;
    int location;
    QString* rule;
};


#endif // SENSORINFO_H
