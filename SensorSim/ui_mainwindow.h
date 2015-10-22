/********************************************************************************
** Form generated from reading UI file 'mainwindow.ui'
**
** Created: Mon Jan 20 16:05:02 2014
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
    QGroupBox *groupBox_2;
    QCheckBox *sofa1CheckBox;
    QCheckBox *fridgeCheckBox;
    QPushButton *pushButton;
    QCheckBox *sofa2CheckBox;
    QCheckBox *sofa3CheckBox;
    QCheckBox *trayCheckBox;
    QCheckBox *cupCheckBox;
    QGroupBox *groupBox_9;
    QLabel *userlabel;
    QGroupBox *groupBox_5;
    QComboBox *userLocationComboBox;
    QGroupBox *groupBox_6;
    QComboBox *robotLocationComboBox;
    QPushButton *pushButton_2;
    QLabel *label_3;
    QLabel *label_4;
    QLabel *runningAtLabel;
    QLabel *label;
    QMenuBar *menuBar;
    QToolBar *mainToolBar;
    QStatusBar *statusBar;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QString::fromUtf8("MainWindow"));
        MainWindow->resize(509, 491);
        centralWidget = new QWidget(MainWindow);
        centralWidget->setObjectName(QString::fromUtf8("centralWidget"));
        groupBox_2 = new QGroupBox(centralWidget);
        groupBox_2->setObjectName(QString::fromUtf8("groupBox_2"));
        groupBox_2->setGeometry(QRect(20, 220, 461, 161));
        sofa1CheckBox = new QCheckBox(groupBox_2);
        sofa1CheckBox->setObjectName(QString::fromUtf8("sofa1CheckBox"));
        sofa1CheckBox->setGeometry(QRect(20, 30, 131, 23));
        fridgeCheckBox = new QCheckBox(groupBox_2);
        fridgeCheckBox->setObjectName(QString::fromUtf8("fridgeCheckBox"));
        fridgeCheckBox->setGeometry(QRect(20, 60, 361, 23));
        pushButton = new QPushButton(groupBox_2);
        pushButton->setObjectName(QString::fromUtf8("pushButton"));
        pushButton->setGeometry(QRect(20, 90, 96, 23));
        sofa2CheckBox = new QCheckBox(groupBox_2);
        sofa2CheckBox->setObjectName(QString::fromUtf8("sofa2CheckBox"));
        sofa2CheckBox->setGeometry(QRect(150, 30, 131, 23));
        sofa3CheckBox = new QCheckBox(groupBox_2);
        sofa3CheckBox->setObjectName(QString::fromUtf8("sofa3CheckBox"));
        sofa3CheckBox->setGeometry(QRect(280, 30, 151, 23));
        trayCheckBox = new QCheckBox(groupBox_2);
        trayCheckBox->setObjectName(QString::fromUtf8("trayCheckBox"));
        trayCheckBox->setGeometry(QRect(20, 120, 241, 23));
        cupCheckBox = new QCheckBox(groupBox_2);
        cupCheckBox->setObjectName(QString::fromUtf8("cupCheckBox"));
        cupCheckBox->setGeometry(QRect(220, 90, 221, 23));
        groupBox_9 = new QGroupBox(centralWidget);
        groupBox_9->setObjectName(QString::fromUtf8("groupBox_9"));
        groupBox_9->setGeometry(QRect(0, 640, 231, 31));
        userlabel = new QLabel(groupBox_9);
        userlabel->setObjectName(QString::fromUtf8("userlabel"));
        userlabel->setGeometry(QRect(20, 0, 161, 16));
        groupBox_5 = new QGroupBox(centralWidget);
        groupBox_5->setObjectName(QString::fromUtf8("groupBox_5"));
        groupBox_5->setGeometry(QRect(20, 80, 461, 61));
        userLocationComboBox = new QComboBox(groupBox_5);
        userLocationComboBox->setObjectName(QString::fromUtf8("userLocationComboBox"));
        userLocationComboBox->setGeometry(QRect(50, 30, 381, 25));
        groupBox_6 = new QGroupBox(centralWidget);
        groupBox_6->setObjectName(QString::fromUtf8("groupBox_6"));
        groupBox_6->setGeometry(QRect(20, 150, 461, 61));
        robotLocationComboBox = new QComboBox(groupBox_6);
        robotLocationComboBox->setObjectName(QString::fromUtf8("robotLocationComboBox"));
        robotLocationComboBox->setGeometry(QRect(50, 30, 381, 25));
        pushButton_2 = new QPushButton(centralWidget);
        pushButton_2->setObjectName(QString::fromUtf8("pushButton_2"));
        pushButton_2->setGeometry(QRect(190, 40, 96, 23));
        label_3 = new QLabel(centralWidget);
        label_3->setObjectName(QString::fromUtf8("label_3"));
        label_3->setGeometry(QRect(50, 10, 421, 21));
        QFont font;
        font.setPointSize(16);
        label_3->setFont(font);
        label_4 = new QLabel(centralWidget);
        label_4->setObjectName(QString::fromUtf8("label_4"));
        label_4->setGeometry(QRect(310, 40, 71, 16));
        runningAtLabel = new QLabel(centralWidget);
        runningAtLabel->setObjectName(QString::fromUtf8("runningAtLabel"));
        runningAtLabel->setGeometry(QRect(400, 40, 71, 16));
        label = new QLabel(centralWidget);
        label->setObjectName(QString::fromUtf8("label"));
        label->setGeometry(QRect(40, 400, 421, 16));
        MainWindow->setCentralWidget(centralWidget);
        menuBar = new QMenuBar(MainWindow);
        menuBar->setObjectName(QString::fromUtf8("menuBar"));
        menuBar->setGeometry(QRect(0, 0, 509, 22));
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
        MainWindow->setWindowTitle(QApplication::translate("MainWindow", "ZUYD Sensor Simulator", 0, QApplication::UnicodeUTF8));
        groupBox_2->setTitle(QApplication::translate("MainWindow", "Sensors", 0, QApplication::UnicodeUTF8));
        sofa1CheckBox->setText(QApplication::translate("MainWindow", "Sofa 1 Occupied", 0, QApplication::UnicodeUTF8));
        fridgeCheckBox->setText(QApplication::translate("MainWindow", "Fridge Door Open/Closed (Closed if ticked)", 0, QApplication::UnicodeUTF8));
        pushButton->setText(QApplication::translate("MainWindow", "Doorbell", 0, QApplication::UnicodeUTF8));
        sofa2CheckBox->setText(QApplication::translate("MainWindow", "Sofa 2 Occupied", 0, QApplication::UnicodeUTF8));
        sofa3CheckBox->setText(QApplication::translate("MainWindow", "Small Sofa Occupied", 0, QApplication::UnicodeUTF8));
        trayCheckBox->setText(QApplication::translate("MainWindow", "Tray Empty/Full  (Full if ticked) ", 0, QApplication::UnicodeUTF8));
        cupCheckBox->setText(QApplication::translate("MainWindow", "Cup Empty/Full (Full if ticked)", 0, QApplication::UnicodeUTF8));
        groupBox_9->setTitle(QString());
        userlabel->setText(QString());
        groupBox_5->setTitle(QApplication::translate("MainWindow", "User Location", 0, QApplication::UnicodeUTF8));
        groupBox_6->setTitle(QApplication::translate("MainWindow", "Robot Location", 0, QApplication::UnicodeUTF8));
        pushButton_2->setText(QApplication::translate("MainWindow", "Refresh", 0, QApplication::UnicodeUTF8));
        label_3->setText(QApplication::translate("MainWindow", "ZUYD/TROYES Apartment Sensor Simulator", 0, QApplication::UnicodeUTF8));
        label_4->setText(QApplication::translate("MainWindow", "Running at:", 0, QApplication::UnicodeUTF8));
        runningAtLabel->setText(QString());
        label->setText(QApplication::translate("MainWindow", "TextLabel", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
