/********************************************************************************
** Form generated from reading UI file 'mainwindow.ui'
**
** Created: Thu Jul 17 10:44:26 2014
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
#include <QtGui/QGroupBox>
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
    QGroupBox *showMeGroupBox;
    QLabel *label;
    QPushButton *showMePushButton;
    QPushButton *showMeStopPushButton;
    QComboBox *showMeComboBox;
    QPushButton *showMeForgetButton;
    QLabel *inProgressLabel1;
    QLabel *dbLabel;
    QPushButton *timerPushButton;
    QPushButton *testPushButton;
    QLabel *label_4;
    QGroupBox *showMeGroupBox_2;
    QLabel *label_2;
    QPushButton *showMePushButton_2;
    QPushButton *showMeStopPushButton_2;
    QComboBox *showMeComboBox_2;
    QPushButton *showMeForgetButton_2;
    QLabel *inProgressLabel2;
    QGroupBox *showMeGroupBox_3;
    QLabel *label_3;
    QPushButton *showMePushButton_3;
    QPushButton *showMeStopPushButton_3;
    QComboBox *showMeComboBox_3;
    QPushButton *showMeForgetButton_3;
    QLabel *inProgressLabel3;
    QMenuBar *menuBar;
    QToolBar *mainToolBar;
    QStatusBar *statusBar;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QString::fromUtf8("MainWindow"));
        MainWindow->resize(1048, 739);
        centralWidget = new QWidget(MainWindow);
        centralWidget->setObjectName(QString::fromUtf8("centralWidget"));
        showMeGroupBox = new QGroupBox(centralWidget);
        showMeGroupBox->setObjectName(QString::fromUtf8("showMeGroupBox"));
        showMeGroupBox->setGeometry(QRect(20, 40, 991, 151));
        label = new QLabel(showMeGroupBox);
        label->setObjectName(QString::fromUtf8("label"));
        label->setGeometry(QRect(50, 10, 401, 41));
        QFont font;
        font.setPointSize(16);
        label->setFont(font);
        showMePushButton = new QPushButton(showMeGroupBox);
        showMePushButton->setObjectName(QString::fromUtf8("showMePushButton"));
        showMePushButton->setGeometry(QRect(50, 60, 361, 61));
        showMePushButton->setFont(font);
        showMePushButton->setAutoFillBackground(false);
        showMePushButton->setStyleSheet(QString::fromUtf8("background-color: rgb(60, 255, 12);"));
        showMeStopPushButton = new QPushButton(showMeGroupBox);
        showMeStopPushButton->setObjectName(QString::fromUtf8("showMeStopPushButton"));
        showMeStopPushButton->setGeometry(QRect(50, 60, 361, 61));
        showMeStopPushButton->setFont(font);
        showMeStopPushButton->setStyleSheet(QString::fromUtf8("background-color: rgb(255, 49, 31);\n"
"color: rgb(240, 255, 242);"));
        showMeComboBox = new QComboBox(showMeGroupBox);
        showMeComboBox->setObjectName(QString::fromUtf8("showMeComboBox"));
        showMeComboBox->setGeometry(QRect(460, 10, 441, 31));
        showMeComboBox->setFont(font);
        showMeComboBox->setEditable(true);
        showMeForgetButton = new QPushButton(showMeGroupBox);
        showMeForgetButton->setObjectName(QString::fromUtf8("showMeForgetButton"));
        showMeForgetButton->setGeometry(QRect(660, 60, 311, 61));
        showMeForgetButton->setFont(font);
        inProgressLabel1 = new QLabel(showMeGroupBox);
        inProgressLabel1->setObjectName(QString::fromUtf8("inProgressLabel1"));
        inProgressLabel1->setGeometry(QRect(430, 70, 311, 31));
        inProgressLabel1->setFont(font);
        dbLabel = new QLabel(centralWidget);
        dbLabel->setObjectName(QString::fromUtf8("dbLabel"));
        dbLabel->setGeometry(QRect(50, 670, 701, 16));
        timerPushButton = new QPushButton(centralWidget);
        timerPushButton->setObjectName(QString::fromUtf8("timerPushButton"));
        timerPushButton->setGeometry(QRect(890, 650, 97, 25));
        testPushButton = new QPushButton(centralWidget);
        testPushButton->setObjectName(QString::fromUtf8("testPushButton"));
        testPushButton->setGeometry(QRect(410, 540, 241, 71));
        testPushButton->setFont(font);
        testPushButton->setStyleSheet(QString::fromUtf8("background-color: rgb(255, 255, 55);"));
        label_4 = new QLabel(centralWidget);
        label_4->setObjectName(QString::fromUtf8("label_4"));
        label_4->setGeometry(QRect(430, 0, 221, 31));
        QFont font1;
        font1.setPointSize(21);
        label_4->setFont(font1);
        showMeGroupBox_2 = new QGroupBox(centralWidget);
        showMeGroupBox_2->setObjectName(QString::fromUtf8("showMeGroupBox_2"));
        showMeGroupBox_2->setGeometry(QRect(20, 210, 991, 151));
        label_2 = new QLabel(showMeGroupBox_2);
        label_2->setObjectName(QString::fromUtf8("label_2"));
        label_2->setGeometry(QRect(50, 10, 401, 41));
        label_2->setFont(font);
        showMePushButton_2 = new QPushButton(showMeGroupBox_2);
        showMePushButton_2->setObjectName(QString::fromUtf8("showMePushButton_2"));
        showMePushButton_2->setGeometry(QRect(50, 60, 361, 61));
        showMePushButton_2->setFont(font);
        showMePushButton_2->setAutoFillBackground(false);
        showMePushButton_2->setStyleSheet(QString::fromUtf8("background-color: rgb(60, 255, 12);"));
        showMeStopPushButton_2 = new QPushButton(showMeGroupBox_2);
        showMeStopPushButton_2->setObjectName(QString::fromUtf8("showMeStopPushButton_2"));
        showMeStopPushButton_2->setGeometry(QRect(50, 60, 361, 61));
        showMeStopPushButton_2->setFont(font);
        showMeStopPushButton_2->setStyleSheet(QString::fromUtf8("background-color: rgb(255, 49, 31);\n"
"color: rgb(240, 255, 242);"));
        showMeComboBox_2 = new QComboBox(showMeGroupBox_2);
        showMeComboBox_2->setObjectName(QString::fromUtf8("showMeComboBox_2"));
        showMeComboBox_2->setGeometry(QRect(460, 10, 441, 31));
        showMeComboBox_2->setFont(font);
        showMeComboBox_2->setEditable(true);
        showMeForgetButton_2 = new QPushButton(showMeGroupBox_2);
        showMeForgetButton_2->setObjectName(QString::fromUtf8("showMeForgetButton_2"));
        showMeForgetButton_2->setGeometry(QRect(660, 60, 311, 61));
        showMeForgetButton_2->setFont(font);
        inProgressLabel2 = new QLabel(showMeGroupBox_2);
        inProgressLabel2->setObjectName(QString::fromUtf8("inProgressLabel2"));
        inProgressLabel2->setGeometry(QRect(430, 70, 311, 31));
        inProgressLabel2->setFont(font);
        showMeGroupBox_3 = new QGroupBox(centralWidget);
        showMeGroupBox_3->setObjectName(QString::fromUtf8("showMeGroupBox_3"));
        showMeGroupBox_3->setGeometry(QRect(20, 370, 991, 151));
        label_3 = new QLabel(showMeGroupBox_3);
        label_3->setObjectName(QString::fromUtf8("label_3"));
        label_3->setGeometry(QRect(50, 10, 401, 41));
        label_3->setFont(font);
        showMePushButton_3 = new QPushButton(showMeGroupBox_3);
        showMePushButton_3->setObjectName(QString::fromUtf8("showMePushButton_3"));
        showMePushButton_3->setGeometry(QRect(50, 60, 361, 61));
        showMePushButton_3->setFont(font);
        showMePushButton_3->setAutoFillBackground(false);
        showMePushButton_3->setStyleSheet(QString::fromUtf8("background-color: rgb(60, 255, 12);"));
        showMeStopPushButton_3 = new QPushButton(showMeGroupBox_3);
        showMeStopPushButton_3->setObjectName(QString::fromUtf8("showMeStopPushButton_3"));
        showMeStopPushButton_3->setGeometry(QRect(50, 60, 361, 61));
        showMeStopPushButton_3->setFont(font);
        showMeStopPushButton_3->setStyleSheet(QString::fromUtf8("background-color: rgb(255, 49, 31);\n"
"color: rgb(240, 255, 242);"));
        showMeComboBox_3 = new QComboBox(showMeGroupBox_3);
        showMeComboBox_3->setObjectName(QString::fromUtf8("showMeComboBox_3"));
        showMeComboBox_3->setGeometry(QRect(460, 10, 441, 31));
        showMeComboBox_3->setFont(font);
        showMeComboBox_3->setEditable(true);
        showMeForgetButton_3 = new QPushButton(showMeGroupBox_3);
        showMeForgetButton_3->setObjectName(QString::fromUtf8("showMeForgetButton_3"));
        showMeForgetButton_3->setGeometry(QRect(660, 60, 311, 61));
        showMeForgetButton_3->setFont(font);
        inProgressLabel3 = new QLabel(showMeGroupBox_3);
        inProgressLabel3->setObjectName(QString::fromUtf8("inProgressLabel3"));
        inProgressLabel3->setGeometry(QRect(430, 70, 311, 31));
        inProgressLabel3->setFont(font);
        MainWindow->setCentralWidget(centralWidget);
        menuBar = new QMenuBar(MainWindow);
        menuBar->setObjectName(QString::fromUtf8("menuBar"));
        menuBar->setGeometry(QRect(0, 0, 1048, 22));
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
        MainWindow->setWindowTitle(QApplication::translate("MainWindow", "Show Me", 0, QApplication::UnicodeUTF8));
        showMeGroupBox->setTitle(QString());
        label->setText(QApplication::translate("MainWindow", "What do you want the robot to remember?", 0, QApplication::UnicodeUTF8));
        showMePushButton->setText(QApplication::translate("MainWindow", "Press Me to start showing the robot", 0, QApplication::UnicodeUTF8));
        showMeStopPushButton->setText(QApplication::translate("MainWindow", "Press Me When You have Finished", 0, QApplication::UnicodeUTF8));
        showMeForgetButton->setText(QApplication::translate("MainWindow", "Make The Robot Forget This", 0, QApplication::UnicodeUTF8));
        inProgressLabel1->setText(QString());
        dbLabel->setText(QApplication::translate("MainWindow", "TextLabel", 0, QApplication::UnicodeUTF8));
        timerPushButton->setText(QApplication::translate("MainWindow", "Advanced", 0, QApplication::UnicodeUTF8));
        testPushButton->setText(QApplication::translate("MainWindow", "Test Me", 0, QApplication::UnicodeUTF8));
        label_4->setText(QApplication::translate("MainWindow", "Show Me", 0, QApplication::UnicodeUTF8));
        showMeGroupBox_2->setTitle(QString());
        label_2->setText(QApplication::translate("MainWindow", "What do you want the robot to remember?", 0, QApplication::UnicodeUTF8));
        showMePushButton_2->setText(QApplication::translate("MainWindow", "Press Me to start showing the robot", 0, QApplication::UnicodeUTF8));
        showMeStopPushButton_2->setText(QApplication::translate("MainWindow", "Press Me When You have Finished", 0, QApplication::UnicodeUTF8));
        showMeForgetButton_2->setText(QApplication::translate("MainWindow", "Make The Robot Forget This", 0, QApplication::UnicodeUTF8));
        inProgressLabel2->setText(QString());
        showMeGroupBox_3->setTitle(QString());
        label_3->setText(QApplication::translate("MainWindow", "What do you want the robot to remember?", 0, QApplication::UnicodeUTF8));
        showMePushButton_3->setText(QApplication::translate("MainWindow", "Press Me to start showing the robot", 0, QApplication::UnicodeUTF8));
        showMeStopPushButton_3->setText(QApplication::translate("MainWindow", "Press Me When You have Finished", 0, QApplication::UnicodeUTF8));
        showMeForgetButton_3->setText(QApplication::translate("MainWindow", "Make The Robot Forget This", 0, QApplication::UnicodeUTF8));
        inProgressLabel3->setText(QString());
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
