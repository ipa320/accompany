/********************************************************************************
** Form generated from reading UI file 'mainwindow.ui'
**
** Created: Mon Jan 20 16:39:30 2014
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
#include <QtGui/QSpinBox>
#include <QtGui/QStatusBar>
#include <QtGui/QToolBar>
#include <QtGui/QWidget>

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QWidget *centralWidget;
    QLabel *label;
    QComboBox *locationComboBox;
    QLabel *label_2;
    QLabel *label_3;
    QComboBox *userComboBox;
    QSpinBox *minuteSpinBox;
    QLabel *label_4;
    QLabel *label_5;
    QLabel *label_6;
    QSpinBox *hourSpinBox;
    QLabel *label_7;
    QLabel *actualTime;
    QLabel *schedulerTime;
    QLabel *label_8;
    QLabel *label_9;
    QLabel *locnLabel;
    QMenuBar *menuBar;
    QToolBar *mainToolBar;
    QStatusBar *statusBar;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QString::fromUtf8("MainWindow"));
        MainWindow->resize(733, 415);
        centralWidget = new QWidget(MainWindow);
        centralWidget->setObjectName(QString::fromUtf8("centralWidget"));
        label = new QLabel(centralWidget);
        label->setObjectName(QString::fromUtf8("label"));
        label->setGeometry(QRect(80, 30, 171, 16));
        QFont font;
        font.setPointSize(14);
        font.setBold(true);
        font.setWeight(75);
        label->setFont(font);
        locationComboBox = new QComboBox(centralWidget);
        locationComboBox->setObjectName(QString::fromUtf8("locationComboBox"));
        locationComboBox->setGeometry(QRect(240, 70, 241, 24));
        label_2 = new QLabel(centralWidget);
        label_2->setObjectName(QString::fromUtf8("label_2"));
        label_2->setGeometry(QRect(150, 80, 91, 16));
        QFont font1;
        font1.setPointSize(12);
        label_2->setFont(font1);
        label_3 = new QLabel(centralWidget);
        label_3->setObjectName(QString::fromUtf8("label_3"));
        label_3->setGeometry(QRect(150, 120, 91, 16));
        label_3->setFont(font1);
        userComboBox = new QComboBox(centralWidget);
        userComboBox->setObjectName(QString::fromUtf8("userComboBox"));
        userComboBox->setGeometry(QRect(240, 110, 241, 24));
        minuteSpinBox = new QSpinBox(centralWidget);
        minuteSpinBox->setObjectName(QString::fromUtf8("minuteSpinBox"));
        minuteSpinBox->setGeometry(QRect(250, 150, 54, 24));
        label_4 = new QLabel(centralWidget);
        label_4->setObjectName(QString::fromUtf8("label_4"));
        label_4->setGeometry(QRect(150, 160, 91, 16));
        label_4->setFont(font1);
        label_5 = new QLabel(centralWidget);
        label_5->setObjectName(QString::fromUtf8("label_5"));
        label_5->setGeometry(QRect(320, 160, 101, 16));
        label_6 = new QLabel(centralWidget);
        label_6->setObjectName(QString::fromUtf8("label_6"));
        label_6->setGeometry(QRect(150, 190, 91, 16));
        label_6->setFont(font1);
        hourSpinBox = new QSpinBox(centralWidget);
        hourSpinBox->setObjectName(QString::fromUtf8("hourSpinBox"));
        hourSpinBox->setGeometry(QRect(250, 180, 54, 24));
        label_7 = new QLabel(centralWidget);
        label_7->setObjectName(QString::fromUtf8("label_7"));
        label_7->setGeometry(QRect(320, 180, 101, 16));
        actualTime = new QLabel(centralWidget);
        actualTime->setObjectName(QString::fromUtf8("actualTime"));
        actualTime->setGeometry(QRect(150, 260, 111, 41));
        QFont font2;
        font2.setPointSize(18);
        actualTime->setFont(font2);
        schedulerTime = new QLabel(centralWidget);
        schedulerTime->setObjectName(QString::fromUtf8("schedulerTime"));
        schedulerTime->setGeometry(QRect(320, 260, 111, 41));
        schedulerTime->setFont(font2);
        label_8 = new QLabel(centralWidget);
        label_8->setObjectName(QString::fromUtf8("label_8"));
        label_8->setGeometry(QRect(150, 230, 91, 16));
        label_9 = new QLabel(centralWidget);
        label_9->setObjectName(QString::fromUtf8("label_9"));
        label_9->setGeometry(QRect(300, 230, 171, 16));
        locnLabel = new QLabel(centralWidget);
        locnLabel->setObjectName(QString::fromUtf8("locnLabel"));
        locnLabel->setGeometry(QRect(340, 330, 361, 20));
        MainWindow->setCentralWidget(centralWidget);
        menuBar = new QMenuBar(MainWindow);
        menuBar->setObjectName(QString::fromUtf8("menuBar"));
        menuBar->setGeometry(QRect(0, 0, 733, 22));
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
        label->setText(QApplication::translate("MainWindow", "Session Control", 0, QApplication::UnicodeUTF8));
        label_2->setText(QApplication::translate("MainWindow", "Location", 0, QApplication::UnicodeUTF8));
        label_3->setText(QApplication::translate("MainWindow", "User", 0, QApplication::UnicodeUTF8));
        label_4->setText(QApplication::translate("MainWindow", "Time Offset", 0, QApplication::UnicodeUTF8));
        label_5->setText(QApplication::translate("MainWindow", "In minutes", 0, QApplication::UnicodeUTF8));
        label_6->setText(QApplication::translate("MainWindow", "Time Offset", 0, QApplication::UnicodeUTF8));
        label_7->setText(QApplication::translate("MainWindow", "In hours", 0, QApplication::UnicodeUTF8));
        actualTime->setText(QApplication::translate("MainWindow", "dfg", 0, QApplication::UnicodeUTF8));
        schedulerTime->setText(QApplication::translate("MainWindow", "dfg", 0, QApplication::UnicodeUTF8));
        label_8->setText(QApplication::translate("MainWindow", "Actual Time", 0, QApplication::UnicodeUTF8));
        label_9->setText(QApplication::translate("MainWindow", "Experimental Time", 0, QApplication::UnicodeUTF8));
        locnLabel->setText(QApplication::translate("MainWindow", "TextLabel", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
