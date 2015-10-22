/********************************************************************************
** Form generated from reading UI file 'mainwindow.ui'
**
** Created: Wed Oct 14 14:38:34 2015
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
#include <QtGui/QDateTimeEdit>
#include <QtGui/QDoubleSpinBox>
#include <QtGui/QHeaderView>
#include <QtGui/QLabel>
#include <QtGui/QMainWindow>
#include <QtGui/QMenuBar>
#include <QtGui/QPlainTextEdit>
#include <QtGui/QPushButton>
#include <QtGui/QSpinBox>
#include <QtGui/QStatusBar>
#include <QtGui/QToolBar>
#include <QtGui/QWidget>

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QWidget *centralWidget;
    QComboBox *sensorComboBox;
    QLabel *label;
    QLabel *locnlabel;
    QPushButton *changePushButton;
    QPushButton *delPushButton;
    QPlainTextEdit *namePlainTextEdit;
    QComboBox *locnComboBox;
    QLabel *label_2;
    QLabel *label_3;
    QComboBox *accessPointComboBox;
    QPlainTextEdit *valuePlainTextEdit;
    QPlainTextEdit *prevValuePlainTextEdit;
    QDateTimeEdit *currentDateTimeEdit;
    QDateTimeEdit *prevDateTimeEdit;
    QLabel *label_4;
    QLabel *label_5;
    QPlainTextEdit *statusPlainTextEdit;
    QPlainTextEdit *prevStatusPlainTextEdit;
    QLabel *label_6;
    QLabel *label_7;
    QLabel *label_8;
    QLabel *label_9;
    QLabel *label_10;
    QPlainTextEdit *channelPlainTextEdit;
    QLabel *label_11;
    QComboBox *sensorTypeComboBox;
    QLabel *label_13;
    QLabel *label_14;
    QLabel *label_15;
    QLabel *label_16;
    QSpinBox *ispinBox;
    QDoubleSpinBox *xSpinBox;
    QDoubleSpinBox *ySpinBox;
    QDoubleSpinBox *oSpinBox;
    QComboBox *ruleComboBox;
    QPushButton *pushButton;
    QPushButton *addPushButton;
    QSpinBox *idSpinBox;
    QLabel *label_12;
    QLabel *label_17;
    QMenuBar *menuBar;
    QToolBar *mainToolBar;
    QStatusBar *statusBar;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QString::fromUtf8("MainWindow"));
        MainWindow->resize(849, 510);
        centralWidget = new QWidget(MainWindow);
        centralWidget->setObjectName(QString::fromUtf8("centralWidget"));
        sensorComboBox = new QComboBox(centralWidget);
        sensorComboBox->setObjectName(QString::fromUtf8("sensorComboBox"));
        sensorComboBox->setGeometry(QRect(110, 10, 251, 25));
        sensorComboBox->setEditable(false);
        label = new QLabel(centralWidget);
        label->setObjectName(QString::fromUtf8("label"));
        label->setGeometry(QRect(20, 10, 62, 16));
        locnlabel = new QLabel(centralWidget);
        locnlabel->setObjectName(QString::fromUtf8("locnlabel"));
        locnlabel->setGeometry(QRect(460, 430, 301, 20));
        changePushButton = new QPushButton(centralWidget);
        changePushButton->setObjectName(QString::fromUtf8("changePushButton"));
        changePushButton->setGeometry(QRect(560, 10, 97, 25));
        delPushButton = new QPushButton(centralWidget);
        delPushButton->setObjectName(QString::fromUtf8("delPushButton"));
        delPushButton->setGeometry(QRect(730, 10, 97, 25));
        namePlainTextEdit = new QPlainTextEdit(centralWidget);
        namePlainTextEdit->setObjectName(QString::fromUtf8("namePlainTextEdit"));
        namePlainTextEdit->setGeometry(QRect(290, 60, 501, 31));
        locnComboBox = new QComboBox(centralWidget);
        locnComboBox->setObjectName(QString::fromUtf8("locnComboBox"));
        locnComboBox->setGeometry(QRect(140, 240, 291, 25));
        label_2 = new QLabel(centralWidget);
        label_2->setObjectName(QString::fromUtf8("label_2"));
        label_2->setGeometry(QRect(21, 240, 91, 20));
        label_3 = new QLabel(centralWidget);
        label_3->setObjectName(QString::fromUtf8("label_3"));
        label_3->setGeometry(QRect(21, 280, 91, 20));
        accessPointComboBox = new QComboBox(centralWidget);
        accessPointComboBox->setObjectName(QString::fromUtf8("accessPointComboBox"));
        accessPointComboBox->setGeometry(QRect(140, 280, 291, 25));
        valuePlainTextEdit = new QPlainTextEdit(centralWidget);
        valuePlainTextEdit->setObjectName(QString::fromUtf8("valuePlainTextEdit"));
        valuePlainTextEdit->setGeometry(QRect(150, 130, 161, 31));
        prevValuePlainTextEdit = new QPlainTextEdit(centralWidget);
        prevValuePlainTextEdit->setObjectName(QString::fromUtf8("prevValuePlainTextEdit"));
        prevValuePlainTextEdit->setGeometry(QRect(150, 180, 161, 31));
        currentDateTimeEdit = new QDateTimeEdit(centralWidget);
        currentDateTimeEdit->setObjectName(QString::fromUtf8("currentDateTimeEdit"));
        currentDateTimeEdit->setGeometry(QRect(520, 130, 194, 24));
        prevDateTimeEdit = new QDateTimeEdit(centralWidget);
        prevDateTimeEdit->setObjectName(QString::fromUtf8("prevDateTimeEdit"));
        prevDateTimeEdit->setGeometry(QRect(520, 180, 194, 24));
        label_4 = new QLabel(centralWidget);
        label_4->setObjectName(QString::fromUtf8("label_4"));
        label_4->setGeometry(QRect(20, 140, 91, 16));
        label_5 = new QLabel(centralWidget);
        label_5->setObjectName(QString::fromUtf8("label_5"));
        label_5->setGeometry(QRect(20, 190, 91, 16));
        statusPlainTextEdit = new QPlainTextEdit(centralWidget);
        statusPlainTextEdit->setObjectName(QString::fromUtf8("statusPlainTextEdit"));
        statusPlainTextEdit->setGeometry(QRect(340, 130, 161, 31));
        prevStatusPlainTextEdit = new QPlainTextEdit(centralWidget);
        prevStatusPlainTextEdit->setObjectName(QString::fromUtf8("prevStatusPlainTextEdit"));
        prevStatusPlainTextEdit->setGeometry(QRect(340, 180, 161, 31));
        label_6 = new QLabel(centralWidget);
        label_6->setObjectName(QString::fromUtf8("label_6"));
        label_6->setGeometry(QRect(210, 110, 91, 16));
        label_7 = new QLabel(centralWidget);
        label_7->setObjectName(QString::fromUtf8("label_7"));
        label_7->setGeometry(QRect(370, 110, 91, 16));
        label_8 = new QLabel(centralWidget);
        label_8->setObjectName(QString::fromUtf8("label_8"));
        label_8->setGeometry(QRect(560, 110, 91, 16));
        label_9 = new QLabel(centralWidget);
        label_9->setObjectName(QString::fromUtf8("label_9"));
        label_9->setGeometry(QRect(20, 330, 91, 16));
        label_10 = new QLabel(centralWidget);
        label_10->setObjectName(QString::fromUtf8("label_10"));
        label_10->setGeometry(QRect(20, 380, 91, 16));
        channelPlainTextEdit = new QPlainTextEdit(centralWidget);
        channelPlainTextEdit->setObjectName(QString::fromUtf8("channelPlainTextEdit"));
        channelPlainTextEdit->setGeometry(QRect(140, 370, 291, 31));
        label_11 = new QLabel(centralWidget);
        label_11->setObjectName(QString::fromUtf8("label_11"));
        label_11->setGeometry(QRect(20, 430, 91, 20));
        sensorTypeComboBox = new QComboBox(centralWidget);
        sensorTypeComboBox->setObjectName(QString::fromUtf8("sensorTypeComboBox"));
        sensorTypeComboBox->setGeometry(QRect(140, 430, 291, 25));
        label_13 = new QLabel(centralWidget);
        label_13->setObjectName(QString::fromUtf8("label_13"));
        label_13->setGeometry(QRect(490, 270, 62, 16));
        label_14 = new QLabel(centralWidget);
        label_14->setObjectName(QString::fromUtf8("label_14"));
        label_14->setGeometry(QRect(590, 270, 62, 16));
        label_15 = new QLabel(centralWidget);
        label_15->setObjectName(QString::fromUtf8("label_15"));
        label_15->setGeometry(QRect(670, 270, 81, 16));
        label_16 = new QLabel(centralWidget);
        label_16->setObjectName(QString::fromUtf8("label_16"));
        label_16->setGeometry(QRect(530, 350, 62, 16));
        ispinBox = new QSpinBox(centralWidget);
        ispinBox->setObjectName(QString::fromUtf8("ispinBox"));
        ispinBox->setGeometry(QRect(580, 340, 52, 24));
        ispinBox->setMaximum(10);
        xSpinBox = new QDoubleSpinBox(centralWidget);
        xSpinBox->setObjectName(QString::fromUtf8("xSpinBox"));
        xSpinBox->setGeometry(QRect(480, 290, 62, 24));
        xSpinBox->setMaximum(360);
        xSpinBox->setSingleStep(0.1);
        ySpinBox = new QDoubleSpinBox(centralWidget);
        ySpinBox->setObjectName(QString::fromUtf8("ySpinBox"));
        ySpinBox->setGeometry(QRect(580, 290, 62, 24));
        ySpinBox->setMaximum(360);
        ySpinBox->setSingleStep(0.1);
        oSpinBox = new QDoubleSpinBox(centralWidget);
        oSpinBox->setObjectName(QString::fromUtf8("oSpinBox"));
        oSpinBox->setGeometry(QRect(670, 290, 62, 24));
        oSpinBox->setMaximum(360);
        oSpinBox->setSingleStep(0.1);
        ruleComboBox = new QComboBox(centralWidget);
        ruleComboBox->setObjectName(QString::fromUtf8("ruleComboBox"));
        ruleComboBox->setGeometry(QRect(140, 330, 291, 25));
        ruleComboBox->setEditable(true);
        pushButton = new QPushButton(centralWidget);
        pushButton->setObjectName(QString::fromUtf8("pushButton"));
        pushButton->setGeometry(QRect(730, 150, 97, 25));
        addPushButton = new QPushButton(centralWidget);
        addPushButton->setObjectName(QString::fromUtf8("addPushButton"));
        addPushButton->setGeometry(QRect(410, 10, 97, 25));
        idSpinBox = new QSpinBox(centralWidget);
        idSpinBox->setObjectName(QString::fromUtf8("idSpinBox"));
        idSpinBox->setGeometry(QRect(90, 70, 81, 24));
        label_12 = new QLabel(centralWidget);
        label_12->setObjectName(QString::fromUtf8("label_12"));
        label_12->setGeometry(QRect(20, 70, 62, 16));
        label_17 = new QLabel(centralWidget);
        label_17->setObjectName(QString::fromUtf8("label_17"));
        label_17->setGeometry(QRect(190, 70, 91, 16));
        MainWindow->setCentralWidget(centralWidget);
        menuBar = new QMenuBar(MainWindow);
        menuBar->setObjectName(QString::fromUtf8("menuBar"));
        menuBar->setGeometry(QRect(0, 0, 849, 22));
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
        MainWindow->setWindowTitle(QApplication::translate("MainWindow", "Sensor Maintenance", 0, QApplication::UnicodeUTF8));
        label->setText(QApplication::translate("MainWindow", "Sensor", 0, QApplication::UnicodeUTF8));
        locnlabel->setText(QApplication::translate("MainWindow", "TextLabel", 0, QApplication::UnicodeUTF8));
        changePushButton->setText(QApplication::translate("MainWindow", "Change", 0, QApplication::UnicodeUTF8));
        delPushButton->setText(QApplication::translate("MainWindow", "Delete", 0, QApplication::UnicodeUTF8));
        label_2->setText(QApplication::translate("MainWindow", "Location", 0, QApplication::UnicodeUTF8));
        label_3->setText(QApplication::translate("MainWindow", "Access Point", 0, QApplication::UnicodeUTF8));
        label_4->setText(QApplication::translate("MainWindow", "Current Value", 0, QApplication::UnicodeUTF8));
        label_5->setText(QApplication::translate("MainWindow", "Previous", 0, QApplication::UnicodeUTF8));
        label_6->setText(QApplication::translate("MainWindow", "Value", 0, QApplication::UnicodeUTF8));
        label_7->setText(QApplication::translate("MainWindow", "Status", 0, QApplication::UnicodeUTF8));
        label_8->setText(QApplication::translate("MainWindow", "Updated at", 0, QApplication::UnicodeUTF8));
        label_9->setText(QApplication::translate("MainWindow", "Rule", 0, QApplication::UnicodeUTF8));
        label_10->setText(QApplication::translate("MainWindow", "Channel", 0, QApplication::UnicodeUTF8));
        label_11->setText(QApplication::translate("MainWindow", "Sensor Type", 0, QApplication::UnicodeUTF8));
        label_13->setText(QApplication::translate("MainWindow", "X-coord", 0, QApplication::UnicodeUTF8));
        label_14->setText(QApplication::translate("MainWindow", "Y-coord", 0, QApplication::UnicodeUTF8));
        label_15->setText(QApplication::translate("MainWindow", "Orientation", 0, QApplication::UnicodeUTF8));
        label_16->setText(QApplication::translate("MainWindow", "Icon", 0, QApplication::UnicodeUTF8));
        pushButton->setText(QApplication::translate("MainWindow", "Switch", 0, QApplication::UnicodeUTF8));
        addPushButton->setText(QApplication::translate("MainWindow", "Add", 0, QApplication::UnicodeUTF8));
        label_12->setText(QApplication::translate("MainWindow", "Number", 0, QApplication::UnicodeUTF8));
        label_17->setText(QApplication::translate("MainWindow", "Sensor name", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
