#ifndef NEWLOCATION_H
#define NEWLOCATION_H

#include <QDialog>
#include <QtSql>

namespace Ui {
class newLocation;
}

class newLocation : public QDialog
{
    Q_OBJECT
    
public:
    explicit newLocation(QSqlDatabase db, QString start, QString end, QWidget *parent = 0 );
    ~newLocation();

    void setDB(QSqlDatabase idb)
    {
        db = idb;
    };

    void setValidLocns(QString startLocation, QString endLocation)
    {
        start=startLocation;
        end=endLocation;
    };

    QString getName();

    int getId(){return nextId;};

    int getValidRobot();

    int  getValidPerson();

    QString  getParent();

    QString  getNearest();




    
private:
    Ui::newLocation *ui;

    QSqlDatabase db;

    QString start, end;

     int nextId;
};

#endif // NEWLOCATION_H
