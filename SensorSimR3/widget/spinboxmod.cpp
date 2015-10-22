#include "spinboxmod.h"
#include "widgethub.h"

Messenger<SpinBoxMod> SpinBoxMod::messenger;

SpinBoxMod::SpinBoxMod(QWidget* parent, bool designerMode) : QSpinBox(parent)
{
    targetUI = NULL;
    initFlag =false;
    this->designerMode = designerMode;
    messenger.addListener(this);
    //connect(this, SIGNAL(valueChanged(int)),this,SLOT(valueChangedHandle(int)));
    connect(this, SIGNAL(editingFinished()),this,SLOT(editingFinishedHandle()));

}
SpinBoxMod::~SpinBoxMod() {
    messenger.removeListener(this);
}

int SpinBoxMod::getGroupId() { return groupId; }
void SpinBoxMod::setGroupId(int v) { groupId = v; }

void SpinBoxMod::setMaximumSize(const QSize &s){
    QSpinBox::setMaximumSize(s);
    messenger.notifyListener(this);
}

QSize SpinBoxMod::maximumSize()
{
    return QSpinBox::maximumSize();
}

void SpinBoxMod::moveEvent(QMoveEvent * ev) {
    QSpinBox::moveEvent(ev);
    if(designerMode || !initFlag) {
        initFlag =true;
        double val = 0;
        if(targetUI != NULL) targetUI->removeListener(this);
        targetUI = NULL;
        //For CheckBoxUI only...
        QList<CheckBoxUI *> foundCheckBoxUIs = this->parentWidget()->findChildren<CheckBoxUI *>();
        if(foundCheckBoxUIs.size() == 1) {
            if(foundCheckBoxUIs.at(0)->isEnabled()) {
                targetUI = foundCheckBoxUIs.at(0);
                val = WidgetHub::getFactory()->getDbValue((targetUI->getSensorId()));
                if(val>-1) {
                    //if(val!=0) foundCheckBoxUIs.at(0)->setChecked(true);
                    if (val==0) {
                        val = WidgetHub::getDbFactory()->getLastValue(targetUI->getSensorId());
                    }
                    targetUI->setSensorValue(val);
                }
            }
        }
        else {
            QList<ButtonUI *> foundButtonUIs = this->parentWidget()->findChildren<ButtonUI *>();
            if(foundButtonUIs.size() == 1)
            {
                if(foundButtonUIs.at(0)->isEnabled()) {
                    targetUI = foundButtonUIs.at(0);
                    if(val>-1) {
                        //if(val!=0) foundCheckBoxUIs.at(0)->setChecked(true);
                        if (val==0) {
                            val = WidgetHub::getDbFactory()->getLastValue(targetUI->getSensorId());
                        }
                        targetUI->setSensorValue(val);
                    }
                }
            }
        }
        if(targetUI == NULL) {
            setEnabled(false);
        }
        else {
            /*
            if(targetUI->getSensorType() == 1 || targetUI->getSensorType() == 2 || targetUI->getSensorType() == 4) {
                int v = WidgetHub::getFactory()->getDbValue((targetUI->getSensorId()));
                targetUI->setSensorValue(v);
                setEnabled(true);
            }
            else {
                setEnabled(false);
            }
            */

            setValue(targetUI->getSensorValue());
            targetUI->addListener(this);
        }
    }
}
/*    void SpinBoxMod::setMaximumSize(int maxw, int maxh) {
    QSpinBox::setMaximumSize(maxw, maxh);
    messenger.notifyHandler(this);
}

void SpinBoxMod::setMaximumWidth(int maxw) {
    QSpinBox::setMaximumWidth(maxw);
    messenger.notifyHandler(this);
}
void SpinBoxMod::setGeometry(const QRect &g){
    QSpinBox::setGeometry(g);
    notifyHandler(this);
}

void SpinBoxMod::setGeometry(int x, int y, int w, int h){
    QSpinBox::setGeometry(x,y,w,h);
    notifyHandler(this);
}
*/
void SpinBoxMod::notificationHandler(SpinBoxMod* o)
{
    if (this != o && o->getGroupId() == groupId ) {
        QSpinBox::setMaximumSize(o->maximumSize());
    }
}
void SpinBoxMod::notificationHandler(ISensorUI* ui)
{
    if(ui->getSensorValue() != this->value())
    {
        this->setValue(ui->getSensorValue());
    }
}

void SpinBoxMod::valueChangedHandle(int i) {
    if(targetUI != NULL){
        if(i > targetUI->getSensorValue())
            targetUI->setSensorValue(i, &getValueIncreasedMessage());
        else
            targetUI->setSensorValue(i, &getValueDecreasedMessage());
    }
}

void SpinBoxMod::editingFinishedHandle() {
    if(targetUI != NULL){
        if(value() > targetUI->getSensorValue())
            targetUI->setSensorValue(value(), &getValueIncreasedMessage());
        else
            targetUI->setSensorValue(value(), &getValueDecreasedMessage());
    }
}

QString SpinBoxMod::getValueIncreasedMessage()
{
    return valueIncreasedMessage;
}

void SpinBoxMod::setValueIncreasedMessage(QString valueIncreasedMessage)
{
    this->valueIncreasedMessage = valueIncreasedMessage;
}

QString SpinBoxMod::getValueDecreasedMessage()
{
    return valueDecreasedMessage;
}

void SpinBoxMod::setValueDecreasedMessage(QString valueDecreasedMessage)
{
    this->valueDecreasedMessage = valueDecreasedMessage;
}
