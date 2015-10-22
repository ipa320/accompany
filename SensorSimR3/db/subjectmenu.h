#ifndef SUBJECTMENU_H
#define SUBJECTMENU_H
#include <QString>
#include <QList>
#include <QComboBox>

class SubjectMenu
{
public:
    enum SubjectType {User, Robot};
    SubjectMenu();
    int id;
    QString* name;
    static QList<SubjectMenu *> getList(SubjectType subjectType);
    static int getSubjectLocation(SubjectType subjectType, int id);
    static void setSubjectLocation(SubjectType subjectType, int id, int location);
    static void addToComboBox(SubjectType subjectType, QComboBox *cbo);
};

#endif // SUBJECTMENU_H
