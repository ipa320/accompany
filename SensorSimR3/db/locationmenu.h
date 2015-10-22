#ifndef LOCATIONMENU_H
#define LOCATIONMENU_H

#include <QString>
#include <QComboBox>
class LocationMenu
{
public:
    LocationMenu();
    enum SubjectType {User, Robot};
    int id;
    int depth;
    long lineage;
    QString* name;
    QString* parent;
    QString* grandParent;
    static QList<LocationMenu *> getList(SubjectType subjectType);
    static void addToComboBox(SubjectType subjectType, QComboBox *cbo);};

#endif // LOCATIONMENU_H
