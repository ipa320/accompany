#include <QtGui/QApplication>
#include "mainwindow.h"

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);

    MainWindow w;


    if (w.closeDownRequest)
    {
        return 0;
    }

    if ( QApplication::desktop()->width() > w.width() + 10
        && QApplication::desktop()->height() > w.height() +30 )
        w.show();
    else
        w.showMaximized();

    
    return a.exec();
}
