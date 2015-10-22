#ifndef QWIDGETMOD_H
#define QWIDGETMOD_H

#include <QWidget>
#include <QString>
#include <QMessageBox>
#include "isensorui.h"

#ifndef plugin
#undef QDESIGNER_WIDGET_EXPORT
#define QDESIGNER_WIDGET_EXPORT
#endif

class QDESIGNER_WIDGET_EXPORT QWidgetMod:
        public QWidget, public Listener<QWidgetMod>
{
    Q_OBJECT
    Q_PROPERTY(int GroupId READ getGroupId WRITE setGroupId)
    Q_PROPERTY(QRect geometry READ geometry WRITE setGeometry)
public:
    explicit QWidgetMod(QWidget* = 0, bool = false);
    ~QWidgetMod();
    void setGeometry(const QRect &g);
    QRect geometry();
/*
    void setGeometry(int x, int y, int w, int h){
        QWidget::setGeometry(x,y,w,h);
        notifyHandler(this);
    }
*/
    void notificationHandler(QWidgetMod* o);

    int getGroupId();
    void setGroupId(int v);

signals:
//    void stateChanged(ISensorUI *);

private slots:
private:
    int groupId;
    static Messenger<QWidgetMod> messenger;
};

#endif // QWIDGETMOD_H
