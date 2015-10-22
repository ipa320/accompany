/********************************************************************************
** Form generated from reading UI file 'mainwindow.ui'
**
** Created: Wed Jun 25 15:53:32 2014
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
#include <QtGui/QComboBox>
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
    QLabel *locnLabel;
    QPushButton *behLogPushButton;
    QComboBox *activityComboBox;
    QLabel *label;
    QPushButton *ruleSetPushButton;
    QPushButton *ruleStatePushButton;
    QPushButton *classUpdatePushButton;
    QPushButton *namesPushButton;
    QMenuBar *menuBar;
    QToolBar *mainToolBar;
    QStatusBar *statusBar;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QString::fromUtf8("MainWindow"));
        MainWindow->resize(893, 595);
        centralWidget = new QWidget(MainWindow);
        centralWidget->setObjectName(QString::fromUtf8("centralWidget"));
        locnLabel = new QLabel(centralWidget);
        locnLabel->setObjectName(QString::fromUtf8("locnLabel"));
        locnLabel->setGeometry(QRect(70, 510, 771, 16));
        behLogPushButton = new QPushButton(centralWidget);
        behLogPushButton->setObjectName(QString::fromUtf8("behLogPushButton"));
        behLogPushButton->setGeometry(QRect(240, 260, 361, 25));
        activityComboBox = new QComboBox(centralWidget);
        activityComboBox->setObjectName(QString::fromUtf8("activityComboBox"));
        activityComboBox->setGeometry(QRect(340, 70, 391, 25));
        label = new QLabel(centralWidget);
        label->setObjectName(QString::fromUtf8("label"));
        label->setGeometry(QRect(100, 70, 221, 20));
        ruleSetPushButton = new QPushButton(centralWidget);
        ruleSetPushButton->setObjectName(QString::fromUtf8("ruleSetPushButton"));
        ruleSetPushButton->setGeometry(QRect(230, 430, 361, 25));
        ruleStatePushButton = new QPushButton(centralWidget);
        ruleStatePushButton->setObjectName(QString::fromUtf8("ruleStatePushButton"));
        ruleStatePushButton->setGeometry(QRect(240, 300, 361, 25));
        classUpdatePushButton = new QPushButton(centralWidget);
        classUpdatePushButton->setObjectName(QString::fromUtf8("classUpdatePushButton"));
        classUpdatePushButton->setGeometry(QRect(230, 480, 361, 25));
        namesPushButton = new QPushButton(centralWidget);
        namesPushButton->setObjectName(QString::fromUtf8("namesPushButton"));
        namesPushButton->setGeometry(QRect(240, 220, 361, 25));
        MainWindow->setCentralWidget(centralWidget);
        menuBar = new QMenuBar(MainWindow);
        menuBar->setObjectName(QString::fromUtf8("menuBar"));
        menuBar->setGeometry(QRect(0, 0, 893, 22));
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
        MainWindow->setWindowTitle(QApplication::translate("MainWindow", "MainWindow", 0, QApplication::UnicodeUTF8));
        locnLabel->setText(QString());
        behLogPushButton->setText(QApplication::translate("MainWindow", "Create Behaviour Log", 0, QApplication::UnicodeUTF8));
        label->setText(QApplication::translate("MainWindow", "Generate Rules for This Activity", 0, QApplication::UnicodeUTF8));
        ruleSetPushButton->setText(QApplication::translate("MainWindow", "Create Rule Set", 0, QApplication::UnicodeUTF8));
        ruleStatePushButton->setText(QApplication::translate("MainWindow", "Create Sensor State history", 0, QApplication::UnicodeUTF8));
        classUpdatePushButton->setText(QApplication::translate("MainWindow", "Update Classes", 0, QApplication::UnicodeUTF8));
        namesPushButton->setText(QApplication::translate("MainWindow", "Create Names File", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
