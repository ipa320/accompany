# -------------------------------------------------
# Project created by QtCreator 2012-07-23T15:39:06
# -------------------------------------------------
QT += sql
TARGET = SensorSim
TEMPLATE = app
SOURCES += main.cpp \
    mainwindow.cpp \
    widget/checkboxui.cpp \
    widget/isensorui.cpp \
    widget/buttonui.cpp \
    widget/widgethub.cpp \
    widget/spinboxmod.cpp \
    widget/isensormod.cpp \
    widget/qwidgetmod.cpp \
    db/dbsvc.cpp \
    widget/sensorevent.cpp \
    db/subjectmenu.cpp \
    db/locationmenu.cpp
HEADERS += mainwindow.h \
    widget/checkboxui.h \
    widget/isensorui.h \
    db/sensorinfo.h \
    widget/buttonui.h \
    widget/widgethub.h \
    widget/spinboxmod.h \
    widget/isensormod.h \
    widget/qwidgetmod.h \
    db/dbsvc.h \
    widget/sensorevent.h \
    db/subjectmenu.h \
    db/locationmenu.h
FORMS += mainwindow.ui
