/********************************************************************************
** Form generated from reading UI file 'mainwindow.ui'
**
** Created: Wed Jan 15 13:14:31 2014
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
#include <QtGui/QPushButton>
#include <QtGui/QRadioButton>
#include <QtGui/QSpinBox>
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
    QCheckBox *showNonSchedcheckBox;
    QPushButton *testPlannerPushButton;
    QPushButton *enableDebugPushButton;
    QLabel *label_2;
    QGroupBox *GUIgroupBox;
    QRadioButton *GUIradioButton1;
    QRadioButton *GUIradioButton2;
    QRadioButton *GUIradioButton3;
    QRadioButton *GUIradioButton4;
    QSpinBox *speedSpinBox;
    QLabel *label_3;
    QLabel *label_4;
    QLabel *sceanrioLabel;
    QToolBar *mainToolBar;
    QStatusBar *statusBar;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QString::fromUtf8("MainWindow"));
        MainWindow->resize(1288, 822);
        centralWidget = new QWidget(MainWindow);
        centralWidget->setObjectName(QString::fromUtf8("centralWidget"));
        sequenceTableWidget = new QTableWidget(centralWidget);
        sequenceTableWidget->setObjectName(QString::fromUtf8("sequenceTableWidget"));
        sequenceTableWidget->setGeometry(QRect(0, 60, 401, 591));
        QSizePolicy sizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(sequenceTableWidget->sizePolicy().hasHeightForWidth());
        sequenceTableWidget->setSizePolicy(sizePolicy);
        sequenceTableWidget->setEditTriggers(QAbstractItemView::NoEditTriggers);
        sequenceTableWidget->setDragDropOverwriteMode(false);
        sequenceTableWidget->setAlternatingRowColors(true);
        sequenceTableWidget->setSelectionMode(QAbstractItemView::SingleSelection);
        sequenceTableWidget->setSelectionBehavior(QAbstractItemView::SelectRows);
        sequenceTableWidget->setHorizontalScrollMode(QAbstractItemView::ScrollPerPixel);
        evaluatePushButton = new QPushButton(centralWidget);
        evaluatePushButton->setObjectName(QString::fromUtf8("evaluatePushButton"));
        evaluatePushButton->setGeometry(QRect(10, 20, 102, 24));
        evaluateAllPushButton = new QPushButton(centralWidget);
        evaluateAllPushButton->setObjectName(QString::fromUtf8("evaluateAllPushButton"));
        evaluateAllPushButton->setGeometry(QRect(130, 20, 102, 24));
        userlabel = new QLabel(centralWidget);
        userlabel->setObjectName(QString::fromUtf8("userlabel"));
        userlabel->setGeometry(QRect(410, 630, 531, 16));
        SQLtableWidget = new QTableWidget(centralWidget);
        SQLtableWidget->setObjectName(QString::fromUtf8("SQLtableWidget"));
        SQLtableWidget->setGeometry(QRect(400, 60, 881, 331));
        SQLtableWidget->setAutoScrollMargin(15);
        SQLtableWidget->setHorizontalScrollMode(QAbstractItemView::ScrollPerPixel);
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
        logTableWidget->setGeometry(QRect(400, 390, 881, 261));
        logTableWidget->setHorizontalScrollMode(QAbstractItemView::ScrollPerPixel);
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
        showNonSchedcheckBox = new QCheckBox(centralWidget);
        showNonSchedcheckBox->setObjectName(QString::fromUtf8("showNonSchedcheckBox"));
        showNonSchedcheckBox->setGeometry(QRect(790, 30, 171, 23));
        testPlannerPushButton = new QPushButton(centralWidget);
        testPlannerPushButton->setObjectName(QString::fromUtf8("testPlannerPushButton"));
        testPlannerPushButton->setGeometry(QRect(430, 670, 102, 24));
        enableDebugPushButton = new QPushButton(centralWidget);
        enableDebugPushButton->setObjectName(QString::fromUtf8("enableDebugPushButton"));
        enableDebugPushButton->setGeometry(QRect(550, 670, 131, 24));
        enableDebugPushButton->setCheckable(true);
        label_2 = new QLabel(centralWidget);
        label_2->setObjectName(QString::fromUtf8("label_2"));
        label_2->setGeometry(QRect(720, 670, 31, 21));
        QFont font;
        font.setPointSize(8);
        label_2->setFont(font);
        GUIgroupBox = new QGroupBox(centralWidget);
        GUIgroupBox->setObjectName(QString::fromUtf8("GUIgroupBox"));
        GUIgroupBox->setGeometry(QRect(750, 670, 191, 21));
        GUIradioButton1 = new QRadioButton(GUIgroupBox);
        GUIradioButton1->setObjectName(QString::fromUtf8("GUIradioButton1"));
        GUIradioButton1->setGeometry(QRect(10, 0, 41, 21));
        GUIradioButton2 = new QRadioButton(GUIgroupBox);
        GUIradioButton2->setObjectName(QString::fromUtf8("GUIradioButton2"));
        GUIradioButton2->setGeometry(QRect(50, 0, 41, 20));
        GUIradioButton3 = new QRadioButton(GUIgroupBox);
        GUIradioButton3->setObjectName(QString::fromUtf8("GUIradioButton3"));
        GUIradioButton3->setGeometry(QRect(100, 0, 41, 21));
        GUIradioButton4 = new QRadioButton(GUIgroupBox);
        GUIradioButton4->setObjectName(QString::fromUtf8("GUIradioButton4"));
        GUIradioButton4->setGeometry(QRect(140, 0, 41, 21));
        speedSpinBox = new QSpinBox(centralWidget);
        speedSpinBox->setObjectName(QString::fromUtf8("speedSpinBox"));
        speedSpinBox->setGeometry(QRect(1150, 670, 71, 24));
        speedSpinBox->setMinimum(100);
        speedSpinBox->setMaximum(3001);
        speedSpinBox->setSingleStep(50);
        label_3 = new QLabel(centralWidget);
        label_3->setObjectName(QString::fromUtf8("label_3"));
        label_3->setGeometry(QRect(960, 670, 181, 16));
        label_4 = new QLabel(centralWidget);
        label_4->setObjectName(QString::fromUtf8("label_4"));
        label_4->setGeometry(QRect(20, 660, 61, 16));
        sceanrioLabel = new QLabel(centralWidget);
        sceanrioLabel->setObjectName(QString::fromUtf8("sceanrioLabel"));
        sceanrioLabel->setGeometry(QRect(80, 660, 221, 16));
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
        showNonSchedcheckBox->setText(QApplication::translate("MainWindow", "Display Non-schedulable ", 0, QApplication::UnicodeUTF8));
        testPlannerPushButton->setText(QApplication::translate("MainWindow", "Test Planner", 0, QApplication::UnicodeUTF8));
        enableDebugPushButton->setText(QApplication::translate("MainWindow", "Enable Debug", 0, QApplication::UnicodeUTF8));
        label_2->setText(QApplication::translate("MainWindow", "GUI", 0, QApplication::UnicodeUTF8));
        GUIgroupBox->setTitle(QString());
        GUIradioButton1->setText(QApplication::translate("MainWindow", "1", 0, QApplication::UnicodeUTF8));
        GUIradioButton2->setText(QApplication::translate("MainWindow", "2", 0, QApplication::UnicodeUTF8));
        GUIradioButton3->setText(QApplication::translate("MainWindow", "3", 0, QApplication::UnicodeUTF8));
        GUIradioButton4->setText(QApplication::translate("MainWindow", "4", 0, QApplication::UnicodeUTF8));
        label_3->setText(QApplication::translate("MainWindow", "Scheduler Loop Speed (ms)", 0, QApplication::UnicodeUTF8));
        label_4->setText(QApplication::translate("MainWindow", "Using -> ", 0, QApplication::UnicodeUTF8));
        sceanrioLabel->setText(QString());
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
