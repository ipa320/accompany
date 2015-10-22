/********************************************************************************
** Form generated from reading UI file 'mainwindow.ui'
**
** Created: Thu Jul 9 10:35:07 2015
**      by: Qt User Interface Compiler version 4.8.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINWINDOW_H
#define UI_MAINWINDOW_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QCheckBox>
#include <QtGui/QGraphicsView>
#include <QtGui/QHeaderView>
#include <QtGui/QLabel>
#include <QtGui/QMainWindow>
#include <QtGui/QMenuBar>
#include <QtGui/QPushButton>
#include <QtGui/QStatusBar>
#include <QtGui/QToolBar>
#include <QtGui/QWidget>

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QWidget *centralWidget;
    QGraphicsView *graphicsView;
    QPushButton *refreshButton;
    QPushButton *updateButton;
    QPushButton *printButton;
    QCheckBox *checkBox;
    QCheckBox *checkBox_2;
    QPushButton *newLocnPushButton;
    QPushButton *zoomInPushButton;
    QPushButton *zoomoutPushButton;
    QPushButton *voronoiPushButton;
    QCheckBox *l0checkBox;
    QCheckBox *l1checkBox;
    QCheckBox *l2checkBox;
    QCheckBox *l3checkBox;
    QLabel *label;
    QMenuBar *menuBar;
    QToolBar *mainToolBar;
    QStatusBar *statusBar;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QString::fromUtf8("MainWindow"));
        MainWindow->resize(1356, 862);
        centralWidget = new QWidget(MainWindow);
        centralWidget->setObjectName(QString::fromUtf8("centralWidget"));
        graphicsView = new QGraphicsView(centralWidget);
        graphicsView->setObjectName(QString::fromUtf8("graphicsView"));
        graphicsView->setGeometry(QRect(30, 0, 1191, 781));
        refreshButton = new QPushButton(centralWidget);
        refreshButton->setObjectName(QString::fromUtf8("refreshButton"));
        refreshButton->setGeometry(QRect(30, 790, 131, 25));
        updateButton = new QPushButton(centralWidget);
        updateButton->setObjectName(QString::fromUtf8("updateButton"));
        updateButton->setGeometry(QRect(170, 790, 191, 25));
        printButton = new QPushButton(centralWidget);
        printButton->setObjectName(QString::fromUtf8("printButton"));
        printButton->setGeometry(QRect(510, 790, 97, 25));
        checkBox = new QCheckBox(centralWidget);
        checkBox->setObjectName(QString::fromUtf8("checkBox"));
        checkBox->setGeometry(QRect(620, 790, 131, 21));
        checkBox_2 = new QCheckBox(centralWidget);
        checkBox_2->setObjectName(QString::fromUtf8("checkBox_2"));
        checkBox_2->setGeometry(QRect(750, 790, 161, 21));
        newLocnPushButton = new QPushButton(centralWidget);
        newLocnPushButton->setObjectName(QString::fromUtf8("newLocnPushButton"));
        newLocnPushButton->setGeometry(QRect(370, 790, 131, 25));
        zoomInPushButton = new QPushButton(centralWidget);
        zoomInPushButton->setObjectName(QString::fromUtf8("zoomInPushButton"));
        zoomInPushButton->setGeometry(QRect(1180, 780, 31, 31));
        zoomoutPushButton = new QPushButton(centralWidget);
        zoomoutPushButton->setObjectName(QString::fromUtf8("zoomoutPushButton"));
        zoomoutPushButton->setGeometry(QRect(1140, 780, 31, 31));
        voronoiPushButton = new QPushButton(centralWidget);
        voronoiPushButton->setObjectName(QString::fromUtf8("voronoiPushButton"));
        voronoiPushButton->setGeometry(QRect(900, 790, 161, 25));
        l0checkBox = new QCheckBox(centralWidget);
        l0checkBox->setObjectName(QString::fromUtf8("l0checkBox"));
        l0checkBox->setGeometry(QRect(1240, 550, 87, 21));
        l1checkBox = new QCheckBox(centralWidget);
        l1checkBox->setObjectName(QString::fromUtf8("l1checkBox"));
        l1checkBox->setGeometry(QRect(1240, 590, 87, 21));
        l2checkBox = new QCheckBox(centralWidget);
        l2checkBox->setObjectName(QString::fromUtf8("l2checkBox"));
        l2checkBox->setGeometry(QRect(1240, 630, 87, 21));
        l3checkBox = new QCheckBox(centralWidget);
        l3checkBox->setObjectName(QString::fromUtf8("l3checkBox"));
        l3checkBox->setGeometry(QRect(1240, 670, 87, 21));
        label = new QLabel(centralWidget);
        label->setObjectName(QString::fromUtf8("label"));
        label->setGeometry(QRect(1230, 500, 111, 31));
        MainWindow->setCentralWidget(centralWidget);
        menuBar = new QMenuBar(MainWindow);
        menuBar->setObjectName(QString::fromUtf8("menuBar"));
        menuBar->setGeometry(QRect(0, 0, 1356, 22));
        MainWindow->setMenuBar(menuBar);
        mainToolBar = new QToolBar(MainWindow);
        mainToolBar->setObjectName(QString::fromUtf8("mainToolBar"));
        MainWindow->addToolBar(Qt::TopToolBarArea, mainToolBar);
        statusBar = new QStatusBar(MainWindow);
        statusBar->setObjectName(QString::fromUtf8("statusBar"));
        MainWindow->setStatusBar(statusBar);

        retranslateUi(MainWindow);

        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QApplication::translate("MainWindow", "Valid Robot Locations", 0, QApplication::UnicodeUTF8));
        refreshButton->setText(QApplication::translate("MainWindow", "Refresh from DB", 0, QApplication::UnicodeUTF8));
        updateButton->setText(QApplication::translate("MainWindow", "Update DB with New Positions", 0, QApplication::UnicodeUTF8));
        printButton->setText(QApplication::translate("MainWindow", "Print", 0, QApplication::UnicodeUTF8));
        checkBox->setText(QApplication::translate("MainWindow", "Enable Text Edit", 0, QApplication::UnicodeUTF8));
        checkBox_2->setText(QApplication::translate("MainWindow", "Enable Orientation", 0, QApplication::UnicodeUTF8));
        newLocnPushButton->setText(QApplication::translate("MainWindow", "Add new Location", 0, QApplication::UnicodeUTF8));
        zoomInPushButton->setText(QApplication::translate("MainWindow", "+", 0, QApplication::UnicodeUTF8));
        zoomoutPushButton->setText(QApplication::translate("MainWindow", "-", 0, QApplication::UnicodeUTF8));
        voronoiPushButton->setText(QApplication::translate("MainWindow", "Add Voronoi Lattice", 0, QApplication::UnicodeUTF8));
        l0checkBox->setText(QApplication::translate("MainWindow", "Level 0", 0, QApplication::UnicodeUTF8));
        l1checkBox->setText(QApplication::translate("MainWindow", "Level 1", 0, QApplication::UnicodeUTF8));
        l2checkBox->setText(QApplication::translate("MainWindow", "Level 2", 0, QApplication::UnicodeUTF8));
        l3checkBox->setText(QApplication::translate("MainWindow", "Level 3", 0, QApplication::UnicodeUTF8));
        label->setText(QApplication::translate("MainWindow", "Include Locations", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
