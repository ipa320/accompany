#include <QtGui/QApplication>
#include "mainwindow.h"
#include "widget/widgethub.h"

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    MainWindow w;
    w.show();
    WidgetHub::getFactory()->reportUnregisteredSensor();
    return a.exec();
}
