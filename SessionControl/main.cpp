#include <QtGui/QApplication>
#include "mainwindow.h"

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    MainWindow w;

    w.lv="UH";

    opterr = 0;
    int c;

    while ((c = getopt (argc, argv, "l:")) != -1)
      switch (c)
      {
         case 'l':
            w.lv = optarg;
            break;
         default:
            w.lv = "UH";
            break;
      }


    w.setup();

    if (w.closeDownRequest)
    {
        return 0;
    }

    w.show();
    return a.exec();
}
