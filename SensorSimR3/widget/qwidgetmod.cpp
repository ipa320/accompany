#include "qwidgetmod.h"

Messenger<QWidgetMod> QWidgetMod::messenger;

QWidgetMod::QWidgetMod(QWidget* parent, bool designerMode) : QWidget(parent)
{
    messenger.addListener(this);
}

QWidgetMod::~QWidgetMod() {
    messenger.removeListener(this);
}
void QWidgetMod::setGeometry(const QRect &g){
    QWidget::setGeometry(g);
    messenger.notifyListener(this);
}
QRect QWidgetMod::geometry()
{
    return QWidget::geometry();
}
/*
void QWidgetMod::setGeometry(int x, int y, int w, int h){
    QWidget::setGeometry(x,y,w,h);
    notifyHandler(this);
}
*/
void QWidgetMod::notificationHandler(QWidgetMod* o)
{
    if (this != o && o->getGroupId() == groupId ) {
        QRect r0 = this->geometry();
        QRect r1 = o->geometry();
        QWidget::setGeometry(r0.x(),r0.y(),r1.width(),r1.height());
    }
}

int QWidgetMod::getGroupId() { return groupId; }
void QWidgetMod::setGroupId(int v) { groupId = v; }
