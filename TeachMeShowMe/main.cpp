#include <QtGui/QApplication>
#include "mainwindow.h"

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    MainWindow w;

    w.param="";

    opterr = 0;
    int c;

    while ((c = getopt (argc, argv, "b")) != -1)

    {

      switch (c)
      {
         case 'b':
          w.param = "BATCH";
            break;
         default:
            w.param = "";
            break;
      }
    }

    if (!w.openDatabase())
    {
        return 0;
    }

    if (w.closeDownRequest)
    {
        return 0;
    }

    w.show();

    return a.exec();

}
