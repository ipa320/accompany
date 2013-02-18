#include "spinboxmodplugin.h"

#include "widget/spinboxmod.h"

#include <QtPlugin>
#include <QInputDialog>


SpinBoxModPlugin::SpinBoxModPlugin(QObject *parent)
     : QObject(parent)
 {
     initialized = false;
 }

 void SpinBoxModPlugin::initialize(QDesignerFormEditorInterface * /* core */)
 {
     if (initialized)
     {
         return;
     }

     initialized = true;
 }

 bool SpinBoxModPlugin::isInitialized() const
 {
     return initialized;
 }

 QWidget *SpinBoxModPlugin::createWidget(QWidget *parent)
 {
     SpinBoxMod* ui = new SpinBoxMod(parent,true);
     return ui;
 }

 QString SpinBoxModPlugin::name() const
 {
     return "SpinBoxMod";
 }

 QString SpinBoxModPlugin::group() const
 {
     return "SensorSim Widget";
 }

 QIcon SpinBoxModPlugin::icon() const
 {
     return QIcon();
 }

 QString SpinBoxModPlugin::toolTip() const
 {
     return "toolTip()";
 }

 QString SpinBoxModPlugin::whatsThis() const
 {
     return "whatsThis()";
 }

 bool SpinBoxModPlugin::isContainer() const
 {
     return false;
 }

 QString SpinBoxModPlugin::domXml() const
 {
     return "<ui language=\"c++\">\n"
            " <widget class=\"SpinBoxMod\" name=\"spinBoxMod\">\n"
            "  <property name=\"GroupId\">\n"
            "    <number>0</number>\n"
            "  </property>\n"
"  <property name=\"ValueDecreasedMessage\">\n"
"    <string>decreasing</string>\n"
"  </property>\n"
"  <property name=\"ValueIncreasedMessage\">\n"
"    <string>increasing</string>\n"
"  </property>\n"
            "  <property name=\"geometry\">\n"
            "   <rect>\n"
            "    <x>0</x>\n"
            "    <y>0</y>\n"
            "    <width>42</width>\n"
            "    <height>22</height>\n"
            "   </rect>\n"
            "  </property>\n"
            "  <property name=\"maximumSize\">\n"
            "   <size>\n"
            "    <width>40</width>\n"
            "    <height>16777215</height>\n"
            "   </size>\n"
            "  </property>\n"
            " </widget>\n"
            "</ui>\n";
 }

 QString SpinBoxModPlugin::includeFile() const
 {
     return "widget/spinboxmod.h";
 }

#ifndef collectionplugin
//Q_EXPORT_PLUGIN2(spinboxmod, SpinBoxModPlugin)
#endif
