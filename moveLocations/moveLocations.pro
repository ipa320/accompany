#-------------------------------------------------
#
# Project created by QtCreator 2015-06-17T14:33:19
#
#-------------------------------------------------

QT       += core gui
QT       += sql

TARGET = moveLocations
TEMPLATE = app


SOURCES += main.cpp\
        mainwindow.cpp \
    locn.cpp \
    newlocation.cpp \
    VoronoiDiagramGenerator.cpp

HEADERS  += mainwindow.h \
    locn.h \
    newlocation.h \
    VoronoiDiagramGenerator.h

FORMS    += mainwindow.ui \
    newlocation.ui
