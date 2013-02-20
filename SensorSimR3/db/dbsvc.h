#ifndef DBSVC_H
#define DBSVC_H
#include "sensorinfo.h"
#include <QtSql>
#include <QHash>
class DbSvc {
public:
    DbSvc();
    static DbSvc* getFactory();
    QSqlDatabase* open();

    double getDbValue(int sensorId);
    double getLastValue(int sensorId);
    SensorInfo* getSensorInfo(int sensorId);
    QHash<int,int>* getSensorList();
    void updateSensorLog(int sensorId, int val, QString* stat);

    void updateRobotLocation(int id, int location);
    void updateUSerLocation(int id, int location);

    QString host, user;

    QSqlDatabase* getDb();

private:
    static DbSvc* dbSvc;
    QSqlDatabase* db;
    QString pw;
};

#endif // DBSVC_H
