#include "locationmenu.h"
#include <QtSql>
#include "dbsvc.h"

LocationMenu::LocationMenu()
{
}

QList<LocationMenu *> LocationMenu::getList(LocationMenu::SubjectType subjectType)
{
    QSqlQuery query;
    QList<LocationMenu *> list;
    QString sql = QString("SELECT t2.name as lev2, t3.name as lev3, t4.name as lev4, t4.locationId as lev4id\
                                  , IF(t4.locationId = 0, 0, IF(t3.locationId = 0,1,IF(t2.locationId = 0,2,3))) as depth\
                                  , IF(t4.locationId = 0, 1000000, IF(t3.locationId = 0,1000000 + t4.locationId*10000,IF(t2.locationId = 0,1000000 + t3.locationId*10000 + t4.locationId*100,1000000 + t2.locationId*10000 + t3.locationId*100+t4.locationId))) as lineage\
                                  FROM Locations AS t1\
                                  LEFT JOIN Locations AS t2 ON t2.where = t1.locationId\
                                  LEFT JOIN Locations AS t3 ON t3.where = t2.locationId\
                                  LEFT JOIN Locations AS t4 ON t4.where = t3.locationId\
                                  WHERE t1.locationId = 0\
                                  and t4.locationId is not null\
                                  and t4.%1 = 1\
                                  order by lineage\
                                  ");

    if(subjectType == User) {
        query.prepare(sql.arg("ValidUserLocation"));
    }
    else {
        query.prepare(sql.arg("ValidRobotLocation"));
    }

    if (!query.exec()) {
        qCritical("Cannot retrieve: %s (%s)",
                  DbSvc::getFactory()->getDb()->lastError().text().toLatin1().data(),
                  qt_error_string().toLocal8Bit().data());
    }
    else {
        while (query.next()){
            LocationMenu *menu = new LocationMenu();
            menu->id = query.value(3).toInt();
            menu->name = new QString(query.value(2).toString());
            menu->depth = query.value(4).toInt();
            menu->lineage = query.value(5).toLongLong();
            if(menu->depth >= 2) {
                menu->parent = new QString(query.value(1).toString());
                menu->grandParent = new QString(query.value(0).toString());
            }
            else if(menu->depth == 1) {
                menu->parent = new QString(query.value(1).toString());
                menu->grandParent = NULL;
            }
            else if(menu->depth == 0) {
                menu->parent = NULL;
                menu->grandParent = NULL;
            }
            list.append(menu);
        }
    }
    return list;
 }

void LocationMenu::addToComboBox(LocationMenu::SubjectType subjectType, QComboBox *cbo) {
    QList<LocationMenu *> userLocations = LocationMenu::getList(subjectType);
    foreach (LocationMenu *p, userLocations)
    {
        QString s = "(" + QString::number(p->id) + ") ";
        int depth = p->depth;
        if (depth == 3) {
            s += *p->grandParent + " >> ";
            --depth;
        }
        if (depth == 2)
            s += *p->parent + " >> ";
        s += *p->name;

        cbo->addItem(s, p->id);
    }
}
