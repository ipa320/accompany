#include "checkboxuiplugin.h"
#include "widget/checkboxui.h"

#include <QtPlugin>
#include <QInputDialog>


CheckBoxUIPlugin::CheckBoxUIPlugin(QObject *parent)
     : QObject(parent)
 {
     initialized = false;
 }

 void CheckBoxUIPlugin::initialize(QDesignerFormEditorInterface * /* core */)
 {
     if (initialized)
     {
         return;
     }

     initialized = true;
 }

 bool CheckBoxUIPlugin::isInitialized() const
 {
     return initialized;
 }

 QWidget *CheckBoxUIPlugin::createWidget(QWidget *parent)
 {
     CheckBoxUI* ui = new CheckBoxUI(parent,true);
     return ui;
 }

 QString CheckBoxUIPlugin::name() const
 {
     return "CheckBoxUI";
 }

 QString CheckBoxUIPlugin::group() const
 {
     return "SensorSim Widget";
 }

 QIcon CheckBoxUIPlugin::icon() const
 {
     return QIcon();
 }

 QString CheckBoxUIPlugin::toolTip() const
 {
     return "toolTip()";
 }

 QString CheckBoxUIPlugin::whatsThis() const
 {
     return "whatsThis()";
 }

 bool CheckBoxUIPlugin::isContainer() const
 {
     return false;
 }

 QString CheckBoxUIPlugin::domXml() const
 {
     return "<ui language=\"c++\">\n"
            " <widget class=\"CheckBoxUI\" name=\"checkBoxUI\">\n"
            "  <property name=\"SensorId\">\n"
            "    <number>-1</number>\n"
            "  </property>\n"
            "  <property name=\"text\">\n"
            "    <string>Sensor Name</string>\n"
            "  </property>\n"
"  <property name=\"SensorType\">\n"
"    <number>0</number>\n"
"  </property>\n"
"  <property name=\"SensorLowMessage\">\n"
"    <string>N/A</string>\n"
"  </property>\n"
"  <property name=\"SensorHighMessage\">\n"
"    <string>N/A</string>\n"
"  </property>\n"
            "  <property name=\"testValue\">\n"
            "    <number>123</number>\n"
            "  </property>\n"
            "  <property name=\"geometry\">\n"
            "   <rect>\n"
            "    <x>0</x>\n"
            "    <y>0</y>\n"
            "    <width>100</width>\n"
            "    <height>17</height>\n"
            "   </rect>\n"
            "  </property>\n"
            " </widget>\n"
            "</ui>\n";
 }

 QString CheckBoxUIPlugin::includeFile() const
 {
     return "widget/checkboxui.h";
 }

#ifndef collectionplugin
//Q_EXPORT_PLUGIN2(checkboxui, CheckBoxUIPlugin)
#endif
