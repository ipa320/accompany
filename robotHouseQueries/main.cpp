#include <QtGui/QApplication>
#include "mainwindow.h"

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    MainWindow w;

    int c;

    w.lv="UH";

    opterr = 0;

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



    w.show();
    return a.exec();
}
