/********************************************************************************
** Form generated from reading UI file 'mainwindow.ui'
**
** Created: Fri Mar 8 10:58:50 2013
**      by: Qt User Interface Compiler version 4.6.2
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
#include <QtGui/QHeaderView>
#include <QtGui/QLabel>
#include <QtGui/QMainWindow>
#include <QtGui/QPushButton>
#include <QtGui/QStatusBar>
#include <QtGui/QTableWidget>
#include <QtGui/QToolBar>
#include <QtGui/QWidget>

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QWidget *centralWidget;
    QTableWidget *sequenceTableWidget;
    QPushButton *evaluatePushButton;
    QPushButton *evaluateAllPushButton;
    QLabel *userlabel;
    QTableWidget *SQLtableWidget;
    QPushButton *executePushButton;
    QPushButton *startSchedulerPushButton;
    QPushButton *stopSchedulerPushButton;
    QLabel *label;
    QTableWidget *logTableWidget;
    QPushButton *COBTestPushButton;
    QLabel *userlabel_2;
    QLabel *userlabel_3;
    QLabel *schedulerTime;
    QLabel *actualTime;
    QPushButton *initialiseAllPushButton;
    QPushButton *stopAllPushButton;
    QPushButton *recoverAllPushButton;
    QCheckBox *showNonSchedcheckBox;
    QPushButton *testPlannerPushButton;
    QToolBar *mainToolBar;
    QStatusBar *statusBar;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QString::fromUtf8("MainWindow"));
        MainWindow->resize(1288, 582);
        centralWidget = new QWidget(MainWindow);
        centralWidget->setObjectName(QString::fromUtf8("centralWidget"));
        sequenceTableWidget = new QTableWidget(centralWidget);
        sequenceTableWidget->setObjectName(QString::fromUtf8("sequenceTableWidget"));
        sequenceTableWidget->setGeometry(QRect(0, 60, 361, 471));
        sequenceTableWidget->setEditTriggers(QAbstractItemView::NoEditTriggers);
        sequenceTableWidget->setDragDropOverwriteMode(false);
        sequenceTableWidget->setAlternatingRowColors(true);
        sequenceTableWidget->setSelectionMode(QAbstractItemView::SingleSelection);
        sequenceTableWidget->setSelectionBehavior(QAbstractItemView::SelectRows);
        evaluatePushButton = new QPushButton(centralWidget);
        evaluatePushButton->setObjectName(QString::fromUtf8("evaluatePushButton"));
        evaluatePushButton->setGeometry(QRect(10, 20, 102, 24));
        evaluateAllPushButton = new QPushButton(centralWidget);
        evaluateAllPushButton->setObjectName(QString::fromUtf8("evaluateAllPushButton"));
        evaluateAllPushButton->setGeometry(QRect(130, 20, 102, 24));
        userlabel = new QLabel(centralWidget);
        userlabel->setObjectName(QString::fromUtf8("userlabel"));
        userlabel->setGeometry(QRect(0, 530, 161, 16));
        SQLtableWidget = new QTableWidget(centralWidget);
        SQLtableWidget->setObjectName(QString::fromUtf8("SQLtableWidget"));
        SQLtableWidget->setGeometry(QRect(360, 60, 921, 331));
        SQLtableWidget->setColumnCount(0);
        executePushButton = new QPushButton(centralWidget);
        executePushButton->setObjectName(QString::fromUtf8("executePushButton"));
        executePushButton->setGeometry(QRect(320, 20, 102, 24));
        startSchedulerPushButton = new QPushButton(centralWidget);
        startSchedulerPushButton->setObjectName(QString::fromUtf8("startSchedulerPushButton"));
        startSchedulerPushButton->setGeometry(QRect(500, 20, 102, 24));
        stopSchedulerPushButton = new QPushButton(centralWidget);
        stopSchedulerPushButton->setObjectName(QString::fromUtf8("stopSchedulerPushButton"));
        stopSchedulerPushButton->setGeometry(QRect(640, 20, 102, 24));
        label = new QLabel(centralWidget);
        label->setObjectName(QString::fromUtf8("label"));
        label->setGeometry(QRect(510, 0, 221, 16));
        logTableWidget = new QTableWidget(centralWidget);
        logTableWidget->setObjectName(QString::fromUtf8("logTableWidget"));
        logTableWidget->setGeometry(QRect(360, 390, 921, 141));
        logTableWidget->setColumnCount(0);
        COBTestPushButton = new QPushButton(centralWidget);
        COBTestPushButton->setObjectName(QString::fromUtf8("COBTestPushButton"));
        COBTestPushButton->setGeometry(QRect(790, 10, 171, 24));
        userlabel_2 = new QLabel(centralWidget);
        userlabel_2->setObjectName(QString::fromUtf8("userlabel_2"));
        userlabel_2->setGeometry(QRect(980, 10, 101, 16));
        userlabel_3 = new QLabel(centralWidget);
        userlabel_3->setObjectName(QString::fromUtf8("userlabel_3"));
        userlabel_3->setGeometry(QRect(980, 30, 91, 16));
        schedulerTime = new QLabel(centralWidget);
        schedulerTime->setObjectName(QString::fromUtf8("schedulerTime"));
        schedulerTime->setGeometry(QRect(1090, 10, 101, 16));
        actualTime = new QLabel(centralWidget);
        actualTime->setObjectName(QString::fromUtf8("actualTime"));
        actualTime->setGeometry(QRect(1090, 30, 101, 16));
        initialiseAllPushButton = new QPushButton(centralWidget);
        initialiseAllPushButton->setObjectName(QString::fromUtf8("initialiseAllPushButton"));
        initialiseAllPushButton->setGeometry(QRect(730, 530, 102, 24));
        stopAllPushButton = new QPushButton(centralWidget);
        stopAllPushButton->setObjectName(QString::fromUtf8("stopAllPushButton"));
        stopAllPushButton->setGeometry(QRect(830, 530, 102, 24));
        recoverAllPushButton = new QPushButton(centralWidget);
        recoverAllPushButton->setObjectName(QString::fromUtf8("recoverAllPushButton"));
        recoverAllPushButton->setGeometry(QRect(930, 530, 102, 24));
        showNonSchedcheckBox = new QCheckBox(centralWidget);
        showNonSchedcheckBox->setObjectName(QString::fromUtf8("showNonSchedcheckBox"));
        showNonSchedcheckBox->setGeometry(QRect(790, 30, 171, 23));
        testPlannerPushButton = new QPushButton(centralWidget);
        testPlannerPushButton->setObjectName(QString::fromUtf8("testPlannerPushButton"));
        testPlannerPushButton->setGeometry(QRect(370, 530, 102, 24));
        MainWindow->setCentralWidget(centralWidget);
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
        MainWindow->setWindowTitle(QApplication::translate("MainWindow", "Care-O-Bot Scheduler", 0, QApplication::UnicodeUTF8));
        evaluatePushButton->setText(QApplication::translate("MainWindow", "Evaluate One", 0, QApplication::UnicodeUTF8));
        evaluateAllPushButton->setText(QApplication::translate("MainWindow", "Evaluate All", 0, QApplication::UnicodeUTF8));
        userlabel->setText(QString());
        executePushButton->setText(QApplication::translate("MainWindow", "Execute One", 0, QApplication::UnicodeUTF8));
        startSchedulerPushButton->setText(QApplication::translate("MainWindow", "Start Scheduler", 0, QApplication::UnicodeUTF8));
        stopSchedulerPushButton->setText(QApplication::translate("MainWindow", "Stop Scheduler", 0, QApplication::UnicodeUTF8));
        label->setText(QApplication::translate("MainWindow", "-----------------   Execute All ------------------", 0, QApplication::UnicodeUTF8));
        COBTestPushButton->setText(QApplication::translate("MainWindow", "Refresh Sequences", 0, QApplication::UnicodeUTF8));
        userlabel_2->setText(QApplication::translate("MainWindow", "Scheduler Time", 0, QApplication::UnicodeUTF8));
        userlabel_3->setText(QApplication::translate("MainWindow", "Actual Time", 0, QApplication::UnicodeUTF8));
        schedulerTime->setText(QString());
        actualTime->setText(QString());
        initialiseAllPushButton->setText(QApplication::translate("MainWindow", "Initialise All", 0, QApplication::UnicodeUTF8));
        stopAllPushButton->setText(QApplication::translate("MainWindow", "Stop All", 0, QApplication::UnicodeUTF8));
        recoverAllPushButton->setText(QApplication::translate("MainWindow", "Recover All", 0, QApplication::UnicodeUTF8));
        showNonSchedcheckBox->setText(QApplication::translate("MainWindow", "Display Non-schedulable ", 0, QApplication::UnicodeUTF8));
        testPlannerPushButton->setText(QApplication::translate("MainWindow", "Test Planner", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
