#include "subjectmenu.h"
#include <QtSql>
#include "dbsvc.h"

SubjectMenu::SubjectMenu()
{
}

QList<SubjectMenu *> SubjectMenu::getList(SubjectMenu::SubjectType subjectType)
{
    QSqlQuery query;
    QList<SubjectMenu *> list;

    if(subjectType == Robot) {
        query.prepare("SELECT robotId, robotName from Robot");
    }
    else {
        query.prepare("SELECT userId, nickName from Users");
    }
    if (!query.exec()) {
        qDebug() << "Cannot retrieve: %s (%s)" << DbSvc::getFactory()->getDb()->lastError().text().toLatin1().data() << qt_error_string().toLocal8Bit().data();
        qCritical("Cannot retrieve: %s (%s)",
                  DbSvc::getFactory()->getDb()->lastError().text().toLatin1().data(),
                  qt_error_string().toLocal8Bit().data());
    }
    else {
        while (query.next()){
            SubjectMenu *menu = new SubjectMenu();
            menu->id = query.value(0).toInt();
            menu->name = new QString(query.value(1).toString());
            list.append(menu);
        }
    }
    return list;
}

int SubjectMenu::getSubjectLocation(SubjectMenu::SubjectType subjectType, int id)
{
    QSqlQuery query;

    if(subjectType == User) {
        query.prepare("SELECT userId as id, locationId from Users where userId=:subjectid");
    }
    else {
        query.prepare("SELECT robotId as id, locationId from Robot where robotId=:subjectid");
    }
    query.bindValue(":subjectid",id);

    if (!query.exec()) {
        qCritical("Cannot retrieve: %s (%s)",
                  DbSvc::getFactory()->getDb()->lastError().text().toLatin1().data(),
                  qt_error_string().toLocal8Bit().data());
        return 0;
    }
    else {
        if(!query.next()) {
            qCritical("Selected object (%i) is not exist", id);
            return 0;
        }
        else {
            return query.value(1).toInt();
        }
    }
}

void SubjectMenu::setSubjectLocation(SubjectMenu::SubjectType subjectType, int id, int location)
{
    QSqlQuery query;

    if(subjectType == User) {
        query.prepare("UPDATE Users set locationId=:locationid where userId=:subjectid");
    }
    else {
        query.prepare("UPDATE Robot set locationId=:locationid where robotId=:subjectid");
    }

    query.bindValue(":subjectid",id);
    query.bindValue(":locationid",location);

    if (!query.exec()) {
        qCritical("Cannot update: %s (%s)",
                  DbSvc::getFactory()->getDb()->lastError().text().toLatin1().data(),
                  qt_error_string().toLocal8Bit().data());
    }
}

void SubjectMenu::addToComboBox(SubjectMenu::SubjectType subjectType, QComboBox *cbo) {
    QList<SubjectMenu *> userList = SubjectMenu::getList(subjectType);
    foreach (SubjectMenu *p, userList)
    {
        cbo->addItem(*p->name,p->id);
    }
}
