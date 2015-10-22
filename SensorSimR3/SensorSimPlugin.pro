# -------------------------------------------------
# Project created by QtCreator 2012-07-23T15:39:06
# -------------------------------------------------
QT       += sql
TARGET = SensorSimPlugin
DESTDIR = $$[QT_INSTALL_PLUGINS]/designer
TEMPLATE = lib
CONFIG += designer \
    plugin \
    collectionplugin \
    release
SOURCES += widget/checkboxui.cpp \
    widget/widgethub.cpp \
    widgetplugin/spinboxmodplugin.cpp \
    widget/spinboxmod.cpp \
    widget/isensormod.cpp \
    widgetplugin/sensorsimcollectionplugin.cpp \
    widgetplugin/checkboxuiplugin.cpp \
    widget/isensorui.cpp \
    widget/buttonui.cpp \
    widget/qwidgetmod.cpp \
    widgetplugin/qwidgetmodplugin.cpp \
    widgetplugin/buttonuiplugin.cpp \
    db/dbsvc.cpp \
    widget/sensorevent.cpp \
    db/subjectmenu.cpp \
    db/locationmenu.cpp
HEADERS += widget/checkboxui.h \
    widget/isensorui.h \
    widget/widgethub.h \
    widgetplugin/spinboxmodplugin.h \
    widget/spinboxmod.h \
    widget/isensormod.h \
    widgetplugin/sensorsimcollectionplugin.h \
    widgetplugin/checkboxuiplugin.h \
    widget/buttonui.h \
    widget/qwidgetmod.h \
    widgetplugin/qwidgetmodplugin.h \
    widgetplugin/buttonuiplugin.h \
    db/sensorinfo.h \
    db/dbsvc.h \
    widget/sensorevent.h \
    db/subjectmenu.h \
    db/locationmenu.h
