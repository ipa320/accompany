#ifndef WIDGETHUB_H
#define WIDGETHUB_H
#include "isensorui.h"
#include <QObject>
#include <QString>
#include "buttonui.h"
#include "checkboxui.h"
#include "db/dbsvc.h"
#include "sensorevent.h"
#include <QHash>

class WidgetHub: public QObject
{
    Q_OBJECT
public:
    WidgetHub();
    static WidgetHub* getFactory();
    static DbSvc* getDbFactory();
    SensorInfo* getSensorInfo(int);
    double getDbValue(int sensorId);
    void reportUnregisteredSensor();

    template<class T> void report(T* ui) {
        //note: compile error if implemented in the .cpp file.
        if(!ui->getDesignerMode())
        {
            //connect(ui, SIGNAL(stateChanged(SensorEvent *)),this, SLOT(stateChangedHandler(SensorEvent *)));
        }
    }
    double check(ISensorUI* ui) {
        if(!ui->getDesignerMode())
        {
            if(counter ==0)
            {
                sensorList = DbSvc::getFactory()->getSensorList();
            }
            qDebug() << "WidgetHub: Sensor counter:" << ++counter << "Id=" << ui->getSensorId();

            QWidget *w = dynamic_cast<QWidget *>(ui);
            if(w) {
                connect(w, SIGNAL(stateChanged(SensorEvent *)),this, SLOT(stateChangedHandler(SensorEvent *)));

                if (uiList.contains(ui->getSensorId()))
                    qDebug() << "WidgetHub: Duplicate sensor: Id=" << ui->getSensorId() << w->objectName();
                else {
                    uiList.insert(ui->getSensorId(),ui);
                    sensorList->remove(ui->getSensorId());
                }
                double v = DbSvc::getFactory()->getDbValue(ui->getSensorId());
                if (v<=-1) qDebug() << "WidgetHub: Invalid sensor: Id=" << ui->getSensorId() << w->objectName();
                return v;

            }
            else {
                qDebug() << "WidgetHub: Typecasting Error: " << ui->getSensorId();
                return -3;
            }

        }
        else return 0;
    }
protected:
    int counter;
    QHash<int,ISensorUI*> uiList;
    QHash<int, int>* sensorList;
    //~WidgetHub();
public slots:
    void stateChangedHandler(SensorEvent *);
private:
    static WidgetHub* factory;
};

#endif // WIDGETHUB_H
