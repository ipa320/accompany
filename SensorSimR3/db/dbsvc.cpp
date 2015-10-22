#include "dbsvc.h"
#include <QString>
#include <QMessageBox>
#include <QInputDialog>

DbSvc* DbSvc::dbSvc;// = &DbSvc(); //error: open() (run by the constructor) not stable yet

DbSvc::DbSvc()
{
    open();
}

DbSvc* DbSvc::getFactory(){
    if (dbSvc == NULL) {
        dbSvc = new DbSvc();
    }
    return dbSvc;
}
QSqlDatabase* DbSvc::open(){
    user = QInputDialog::getText ( NULL, "Accompany DB", "User:");
    pw = QInputDialog::getText ( NULL, "Accompany DB", "Password:", QLineEdit::Password);
    host = QInputDialog::getText ( NULL, "Accompany DB", "Host:");


    if (host == "") host = "localhost";
    //if (user == "") user = "accompany";
    if (user == "") user = "root";
    if (pw=="")     pw = "liverp00l";

    //ui->userlabel->setText(user + ":" + host);

    db = & QSqlDatabase::addDatabase("QMYSQL");

    db->setHostName(host);
    db->setDatabaseName("Accompany");
    db->setUserName(user);
    db->setPassword(pw);

    if (db->open()) {
        qDebug() << "Database Opened";
    }
    else {
        QMessageBox::critical(NULL,"Error", "Database error - login problem - see console log!");
        qCritical("Cannot open database: %s (%s)",
                  db->lastError().text().toLatin1().data(),
                  qt_error_string().toLocal8Bit().data());
    }
    return db;
}

void DbSvc::updateSensorLog(int sensorId, int value, QString* stat)
{
    QSqlQuery query;

    query.prepare("INSERT INTO SensorLog (timestamp, sensorId, room, channel, value, status )\
                  VALUES(NOW(), :sensorId, :room, :channel, :value, :status) ");

    query.bindValue(":sensorId",sensorId);
    query.bindValue(":room","");
    query.bindValue(":channel","");
    query.bindValue(":value",value);
    query.bindValue(":status",*stat);

    if (!query.exec())
    {
        QMessageBox::critical(NULL,"Error", "Database error - can't update sensorLog table!");
        qCritical("Cannot insert: %s (%s)",
                  db->lastError().text().toLatin1().data(),
                  qt_error_string().toLocal8Bit().data());

        //qDebug() << query.executedQuery();

        return;
    }
}

double DbSvc::getDbValue(int sensorId) {
    QSqlQuery query;

    query.prepare("SELECT value FROM Sensors WHERE sensorId = :sensorId");
    query.bindValue(":sensorId",sensorId);

    if (!query.exec())
    {
        qCritical("Cannot SELECT: %s (%s)",
                  db->lastError().text().toLatin1().data(),
                  qt_error_string().toLocal8Bit().data());

        //qDebug() << query.executedQuery();
        return -2;
    }
    else {
        if(!query.next()){
            qDebug() << "Sensor not found: " << sensorId;
            return -1;
        }
        else {
            return query.value(0).toDouble();
        }
    }
}

double DbSvc::getLastValue(int sensorId) {
    QSqlQuery query;

    query.prepare("SELECT value FROM SensorLog WHERE value != 0 AND sensorId = :sensorId ORDER BY timestamp DESC LIMIT 1");
    query.bindValue(":sensorId",sensorId);

    if (!query.exec())
    {
        qCritical("Cannot SELECT: %s (%s)",
                  db->lastError().text().toLatin1().data(),
                  qt_error_string().toLocal8Bit().data());

        //qDebug() << query.executedQuery();
        return -2;
    }
    else {
        if(!query.next()){
            return 0; // no last 'on' value
        }
        else {
            return query.value(0).toDouble();
        }
    }
}

SensorInfo* DbSvc::getSensorInfo(int sensorId)
{
    QSqlQuery query;

    SensorInfo* si = new SensorInfo();
    query.prepare("SELECT name, sensorTypeId, locationID, sensorRule FROM Sensors WHERE sensorId = :sensorId");
    query.bindValue(":sensorId",sensorId);
    if (!query.exec()) {
        qCritical("Cannot SELECT: %s (%s)",
                  db->lastError().text().toLatin1().data(),
                  qt_error_string().toLocal8Bit().data());

        //qDebug() << query.executedQuery();
    }
    else {
        if(!query.next()){
            qDebug() << "Sensor not found: " << sensorId;
        }
        else {
            si->name = new QString(query.value(0).toString());
            si->type = query.value(1).toInt();
            si->location = query.value(2).toInt();
            si->rule = new QString(query.value(3).toString());
            return si;
        }
    }
    si->name = new QString("Unknown Sensor");
    si->type = -1;
    si->location = -1;
    si->rule = new QString("Unknown Rule");;
    return si;
}
QHash<int,int> *DbSvc::getSensorList()
{
    QHash<int,int>* sensorList = new QHash<int,int>();
    QSqlQuery query;

    query.prepare("SELECT sensorId, locationId FROM Sensors");

    if (!query.exec())
    {
        qCritical("Cannot SELECT: %s (%s)",
                  db->lastError().text().toLatin1().data(),
                  qt_error_string().toLocal8Bit().data());

        //qDebug() << query.executedQuery();
        return sensorList;
    }
    else {
        while(query.next()) {
            //qDebug() << "DbSvc: Sensor List: " << query.value(0).toInt() << query.value(1).toInt();
            sensorList->insert(query.value(0).toInt(),query.value(1).toInt());
        }
        return sensorList;
    }

}

QSqlDatabase *DbSvc::getDb()
{
    return db;
}
