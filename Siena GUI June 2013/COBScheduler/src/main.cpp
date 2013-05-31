#include <QtGui/QApplication>
#include "../include/COBScheduler/mainwindow.h"

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    MainWindow w(argc, argv);

    if (w.closeDownRequest)
    {
        return 0;
    }

    w.show();
    return a.exec();
}
