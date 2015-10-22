#include <QtGui/QApplication>
#include "mainwindow.h"
#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);

    int c;

    MainWindow w;

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

    w.openbDB();
    if (w.closeDownRequest) return 0;

    w.setup();
    w.show();
    return a.exec();
}
