#ifndef SPINBOXMOD_H
#define SPINBOXMOD_H
#include <QSpinBox>
#include <QString>
#include "isensorui.h"
#include <QMessageBox>
#include "checkboxui.h"
#include <QMoveEvent>

#ifndef plugin
#undef QDESIGNER_WIDGET_EXPORT
#define QDESIGNER_WIDGET_EXPORT
#endif

class QDESIGNER_WIDGET_EXPORT SpinBoxMod:
        public QSpinBox, public Listener<SpinBoxMod>, public Listener<ISensorUI>
{
    Q_OBJECT
    Q_PROPERTY(int GroupId READ getGroupId WRITE setGroupId)
    Q_PROPERTY(QString ValueIncreasedMessage READ getValueIncreasedMessage WRITE setValueIncreasedMessage)
    Q_PROPERTY(QString ValueDecreasedMessage READ getValueDecreasedMessage WRITE setValueDecreasedMessage)
    Q_PROPERTY(QSize maximumSize READ maximumSize WRITE setMaximumSize)
public:
    explicit SpinBoxMod(QWidget* = 0, bool = false);
    ~SpinBoxMod();
    void setMaximumSize(const QSize &s);
    QSize maximumSize();
    void moveEvent(QMoveEvent * ev);
    QString getValueIncreasedMessage();
    void setValueIncreasedMessage(QString valueIncreasedMessage);
    QString getValueDecreasedMessage();
    void setValueDecreasedMessage(QString valueDecreasedMessage);


    /*    void setMaximumSize(int maxw, int maxh) {
        QSpinBox::setMaximumSize(maxw, maxh);
        messenger.notifyHandler(this);
    }

    void setMaximumWidth(int maxw) {
        QSpinBox::setMaximumWidth(maxw);
        messenger.notifyHandler(this);
    }
    void setGeometry(const QRect &g){
        QSpinBox::setGeometry(g);
        notifyHandler(this);
    }

    void setGeometry(int x, int y, int w, int h){
        QSpinBox::setGeometry(x,y,w,h);
        notifyHandler(this);
    }
*/
    void notificationHandler(SpinBoxMod* o);
    void notificationHandler(ISensorUI* ui);

    int getGroupId();
    void setGroupId(int v);

signals:
//    void stateChanged(ISensorUI *);

private slots:
    void valueChangedHandle(int i);
    void editingFinishedHandle();

private:
    int groupId;
    static Messenger<SpinBoxMod> messenger;
    bool initFlag;
    bool designerMode;
    QString valueIncreasedMessage;
    QString valueDecreasedMessage;
    //CheckBoxUI* targetUI;
    ISensorUI* targetUI;
};

#endif // SPINBOXMOD_H
