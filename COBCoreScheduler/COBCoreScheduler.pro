#-------------------------------------------------
#
# Project created by QtCreator 2013-06-28T14:49:36
#
#-------------------------------------------------

QT       += sql

TARGET = COBCoreScheduler
TEMPLATE = app


SOURCES += main.cpp\
        mainwindow.cpp

HEADERS  += mainwindow.h

FORMS    += mainwindow.ui


INCLUDEPATH +=  /home/accompany/git/uh-adapsys-clone-3008/accompany/UHCore/Core/cpp
INCLUDEPATH +=  /usr/include/python2.7/

LIBS += -L/home/accompany/git/uh-adapsys-clone-3008/accompany/UHCore/CppInterface/Debug -lUHCore
LIBS += -lpython2.7


