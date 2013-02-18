#ifndef WidgetHub_H
#define WidgetHub_H
#include "isensorui.h"
#include <QObject>
#include <QString>
#include "buttonui.h"
#include "checkboxui.h"
class WidgetHub: public QObject
{
    Q_OBJECT
public:
    WidgetHub();
    static WidgetHub* getFactory() {
        return factory;
    }
    SensorInfo* getSensorInfo(int);
    template<class T> void report(T* ui) {
        //note: compile error if implemented in the .cpp file.
        connect(ui, SIGNAL(valueChanged(ISensorUI *)),this, SLOT(valueChangedHandler(ISensorUI *)));
    }

private slots:
    void valueChangedHandler(ISensorUI *);
private:
    static WidgetHub* factory;
};

#endif // WidgetHub_H
