#include "sensorsimcollectionplugin.h"
#include "checkboxuiplugin.h"
#include "buttonuiplugin.h"
#include "spinboxmodplugin.h"
#include "qwidgetmodplugin.h"

#include <QtPlugin>
SensorSimCollectionPlugin::SensorSimCollectionPlugin(QObject *parent) :
    QObject(parent)
{
    widgets.append(new CheckBoxUIPlugin(this));
    widgets.append(new ButtonUIPlugin(this));
    widgets.append(new SpinBoxModPlugin(this));
    widgets.append(new QWidgetModPlugin(this));
}

QList<QDesignerCustomWidgetInterface*> SensorSimCollectionPlugin::customWidgets() const
{
    return widgets;
}

Q_EXPORT_PLUGIN2(sensorsimcollectionplugin, SensorSimCollectionPlugin)
