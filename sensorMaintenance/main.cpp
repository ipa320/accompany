#include <QtGui/QApplication>
#include "mainwindow.h"

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    MainWindow w;

    w.setup();

    if (w.closeDownRequest)
    {
        return 0;
    }

    w.show();
    
    return a.exec();
}
