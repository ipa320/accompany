#include "buttonuiplugin.h"
#include "widget/buttonui.h"

#include <QtPlugin>
#include <QInputDialog>


ButtonUIPlugin::ButtonUIPlugin(QObject *parent)
     : QObject(parent)
 {
     initialized = false;
 }

 void ButtonUIPlugin::initialize(QDesignerFormEditorInterface * /* core */)
 {
     if (initialized)
     {
         return;
     }

     initialized = true;
 }

 bool ButtonUIPlugin::isInitialized() const
 {
     return initialized;
 }

 QWidget *ButtonUIPlugin::createWidget(QWidget *parent)
 {
     ButtonUI* ui = new ButtonUI(parent,true);
     return ui;
 }

 QString ButtonUIPlugin::name() const
 {
     return "ButtonUI";
 }

 QString ButtonUIPlugin::group() const
 {
     return "SensorSim Widget";
 }

 QIcon ButtonUIPlugin::icon() const
 {
     return QIcon();
 }

 QString ButtonUIPlugin::toolTip() const
 {
     return "toolTip()";
 }

 QString ButtonUIPlugin::whatsThis() const
 {
     return "whatsThis()";
 }

 bool ButtonUIPlugin::isContainer() const
 {
     return false;
 }

 QString ButtonUIPlugin::domXml() const
 {
     return "<ui language=\"c++\">\n"
            " <widget class=\"ButtonUI\" name=\"buttonUI\">\n"
            "  <property name=\"SensorId\">\n"
            "    <number>-1</number>\n"
            "  </property>\n"
            "  <property name=\"text\">\n"
            "    <string>Sensor Name</string>\n"
            "  </property>\n"
"  <property name=\"SensorLowMessage\">\n"
"    <string>depressed</string>\n"
"  </property>\n"
"  <property name=\"SensorHighMessage\">\n"
"    <string>pressed</string>\n"
"  </property>\n"
            "  <property name=\"testValue\">\n"
            "    <number>123</number>\n"
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

 QString ButtonUIPlugin::includeFile() const
 {
     return "widget/buttonui.h";
 }

#ifndef collectionplugin
//Q_EXPORT_PLUGIN2(buttonui, ButtonUIPlugin)
#endif
