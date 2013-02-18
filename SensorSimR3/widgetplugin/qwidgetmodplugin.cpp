#include "qwidgetmodplugin.h"
#include "widget/qwidgetmod.h"

#include <QtPlugin>
#include <QInputDialog>


QWidgetModPlugin::QWidgetModPlugin(QObject *parent)
     : QObject(parent)
 {
     initialized = false;
 }

 void QWidgetModPlugin::initialize(QDesignerFormEditorInterface * /* core */)
 {
     if (initialized)
     {
         return;
     }

     initialized = true;
 }

 bool QWidgetModPlugin::isInitialized() const
 {
     return initialized;
 }

 QWidget *QWidgetModPlugin::createWidget(QWidget *parent)
 {
     QWidgetMod* ui = new QWidgetMod(parent,true);
     return ui;
 }

 QString QWidgetModPlugin::name() const
 {
     return "QWidgetMod";
 }

 QString QWidgetModPlugin::group() const
 {
     return "SensorSim Widget";
 }

 QIcon QWidgetModPlugin::icon() const
 {
     return QIcon();
 }

 QString QWidgetModPlugin::toolTip() const
 {
     return "toolTip()";
 }

 QString QWidgetModPlugin::whatsThis() const
 {
     return "whatsThis()";
 }

 bool QWidgetModPlugin::isContainer() const
 {
     return true;
 }

 QString QWidgetModPlugin::domXml() const
 {
     return "<ui language=\"c++\">\n"
            " <widget class=\"QWidgetMod\" name=\"qWidgetMod\">\n"
            "  <property name=\"GroupId\">\n"
            "    <number>-1</number>\n"
            "  </property>\n"
            "  <property name=\"geometry\">\n"
            "   <rect>\n"
            "    <x>0</x>\n"
            "    <y>0</y>\n"
            "    <width>100</width>\n"
            "    <height>23</height>\n"
            "   </rect>\n"
            "  </property>\n"
            " </widget>\n"
            "</ui>\n";
 }

 QString QWidgetModPlugin::includeFile() const
 {
     return "widget/qwidgetmod.h";
 }

#ifndef collectionplugin
//Q_EXPORT_PLUGIN2(qwidgetmod, QWidgetModPlugin)
#endif
