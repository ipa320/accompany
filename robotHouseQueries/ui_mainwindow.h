/********************************************************************************
** Form generated from reading UI file 'mainwindow.ui'
**
** Created: Fri Jan 17 11:58:35 2014
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
#include <QtGui/QGroupBox>
#include <QtGui/QHeaderView>
#include <QtGui/QLabel>
#include <QtGui/QMainWindow>
#include <QtGui/QMenuBar>
#include <QtGui/QPushButton>
#include <QtGui/QStatusBar>
#include <QtGui/QTableView>
#include <QtGui/QToolBar>
#include <QtGui/QWidget>

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QWidget *centralWidget;
    QPushButton *openDBButton;
    QPushButton *locationsButton;
    QTableView *locationsView;
    QPushButton *objectsButton;
    QPushButton *peopleButton;
    QPushButton *robotButton;
    QLabel *Userlabel;
    QLabel *serverlabel;
    QGroupBox *groupBox;
    QPushButton *sensorsButton;
    QPushButton *sensorLogButton;
    QPushButton *sensorDefaultsButton;
    QPushButton *emptyLogButton;
    QCheckBox *sensorCheckBox;
    QCheckBox *refreshButton;
    QLabel *locnLabel;
    QMenuBar *menuBar;
    QToolBar *mainToolBar;
    QStatusBar *statusBar;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QString::fromUtf8("MainWindow"));
        MainWindow->resize(950, 773);
        centralWidget = new QWidget(MainWindow);
        centralWidget->setObjectName(QString::fromUtf8("centralWidget"));
        openDBButton = new QPushButton(centralWidget);
        openDBButton->setObjectName(QString::fromUtf8("openDBButton"));
        openDBButton->setGeometry(QRect(20, 0, 201, 24));
        locationsButton = new QPushButton(centralWidget);
        locationsButton->setObjectName(QString::fromUtf8("locationsButton"));
        locationsButton->setGeometry(QRect(20, 30, 111, 24));
        locationsView = new QTableView(centralWidget);
        locationsView->setObjectName(QString::fromUtf8("locationsView"));
        locationsView->setGeometry(QRect(25, 130, 891, 581));
        locationsView->setSortingEnabled(false);
        objectsButton = new QPushButton(centralWidget);
        objectsButton->setObjectName(QString::fromUtf8("objectsButton"));
        objectsButton->setGeometry(QRect(140, 30, 102, 24));
        peopleButton = new QPushButton(centralWidget);
        peopleButton->setObjectName(QString::fromUtf8("peopleButton"));
        peopleButton->setGeometry(QRect(250, 30, 102, 24));
        robotButton = new QPushButton(centralWidget);
        robotButton->setObjectName(QString::fromUtf8("robotButton"));
        robotButton->setGeometry(QRect(360, 30, 102, 24));
        Userlabel = new QLabel(centralWidget);
        Userlabel->setObjectName(QString::fromUtf8("Userlabel"));
        Userlabel->setGeometry(QRect(320, 0, 151, 16));
        serverlabel = new QLabel(centralWidget);
        serverlabel->setObjectName(QString::fromUtf8("serverlabel"));
        serverlabel->setGeometry(QRect(460, 0, 141, 16));
        groupBox = new QGroupBox(centralWidget);
        groupBox->setObjectName(QString::fromUtf8("groupBox"));
        groupBox->setGeometry(QRect(500, 20, 431, 91));
        groupBox->setFlat(false);
        sensorsButton = new QPushButton(groupBox);
        sensorsButton->setObjectName(QString::fromUtf8("sensorsButton"));
        sensorsButton->setGeometry(QRect(30, 10, 141, 24));
        sensorLogButton = new QPushButton(groupBox);
        sensorLogButton->setObjectName(QString::fromUtf8("sensorLogButton"));
        sensorLogButton->setGeometry(QRect(180, 10, 102, 24));
        sensorDefaultsButton = new QPushButton(groupBox);
        sensorDefaultsButton->setObjectName(QString::fromUtf8("sensorDefaultsButton"));
        sensorDefaultsButton->setGeometry(QRect(290, 10, 121, 24));
        emptyLogButton = new QPushButton(groupBox);
        emptyLogButton->setObjectName(QString::fromUtf8("emptyLogButton"));
        emptyLogButton->setGeometry(QRect(290, 40, 121, 24));
        sensorCheckBox = new QCheckBox(groupBox);
        sensorCheckBox->setObjectName(QString::fromUtf8("sensorCheckBox"));
        sensorCheckBox->setGeometry(QRect(80, 50, 171, 21));
        refreshButton = new QCheckBox(groupBox);
        refreshButton->setObjectName(QString::fromUtf8("refreshButton"));
        refreshButton->setGeometry(QRect(80, 70, 121, 21));
        locnLabel = new QLabel(centralWidget);
        locnLabel->setObjectName(QString::fromUtf8("locnLabel"));
        locnLabel->setGeometry(QRect(250, 0, 53, 14));
        MainWindow->setCentralWidget(centralWidget);
        menuBar = new QMenuBar(MainWindow);
        menuBar->setObjectName(QString::fromUtf8("menuBar"));
        menuBar->setGeometry(QRect(0, 0, 950, 20));
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
        MainWindow->setWindowTitle(QApplication::translate("MainWindow", "Robot House Queries ACCOMPANY", 0, QApplication::UnicodeUTF8));
        openDBButton->setText(QApplication::translate("MainWindow", "Open Accompany DB", 0, QApplication::UnicodeUTF8));
        locationsButton->setText(QApplication::translate("MainWindow", "Locations", 0, QApplication::UnicodeUTF8));
        objectsButton->setText(QApplication::translate("MainWindow", "Objects", 0, QApplication::UnicodeUTF8));
        peopleButton->setText(QApplication::translate("MainWindow", "People", 0, QApplication::UnicodeUTF8));
        robotButton->setText(QApplication::translate("MainWindow", "Robots", 0, QApplication::UnicodeUTF8));
        Userlabel->setText(QString());
        serverlabel->setText(QString());
        sensorsButton->setText(QApplication::translate("MainWindow", "Sensors", 0, QApplication::UnicodeUTF8));
        sensorLogButton->setText(QApplication::translate("MainWindow", "Sensor Log", 0, QApplication::UnicodeUTF8));
        sensorDefaultsButton->setText(QApplication::translate("MainWindow", "Default Sensors", 0, QApplication::UnicodeUTF8));
        emptyLogButton->setText(QApplication::translate("MainWindow", "Clear Sensor Log", 0, QApplication::UnicodeUTF8));
        sensorCheckBox->setText(QApplication::translate("MainWindow", "In the Last 60 seconds", 0, QApplication::UnicodeUTF8));
        refreshButton->setText(QApplication::translate("MainWindow", "Auto refresh", 0, QApplication::UnicodeUTF8));
        locnLabel->setText(QString());
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
