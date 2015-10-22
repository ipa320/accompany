#-------------------------------------------------
#
# Project created by QtCreator 2013-06-28T14:49:36
#
#-------------------------------------------------

QT       += sql

    QMAKE_CXXFLAGS_RELEASE -= -O2
    QMAKE_CXXFLAGS_RELEASE += -O1
    QMAKE_CXXFLAGS_RELEASE += -g


TARGET = COBCoreScheduler
TEMPLATE = app


SOURCES += main.cpp\
        mainwindow.cpp

HEADERS  += mainwindow.h

FORMS    += mainwindow.ui


INCLUDEPATH +=  ../UHCore/CppInterface/include
INCLUDEPATH +=  /usr/include/python2.7/

LIBS += -L../UHCore/CppInterface/Debug -lUHCore
LIBS += -lpython2.7




