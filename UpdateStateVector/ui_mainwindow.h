/********************************************************************************
** Form generated from reading UI file 'mainwindow.ui'
**
** Created: Wed Jun 25 15:53:28 2014
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
#include <QtGui/QDoubleSpinBox>
#include <QtGui/QHeaderView>
#include <QtGui/QLCDNumber>
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
    QLabel *locnlabel;
    QLabel *label;
    QLabel *label_2;
    QPushButton *startPushButton;
    QPushButton *stopPushButton;
    QLabel *label_3;
    QLCDNumber *lcdNumber;
    QLabel *label_4;
    QDoubleSpinBox *spinBox;
    QMenuBar *menuBar;
    QToolBar *mainToolBar;
    QStatusBar *statusBar;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QString::fromUtf8("MainWindow"));
        MainWindow->resize(422, 215);
        centralWidget = new QWidget(MainWindow);
        centralWidget->setObjectName(QString::fromUtf8("centralWidget"));
        locnlabel = new QLabel(centralWidget);
        locnlabel->setObjectName(QString::fromUtf8("locnlabel"));
        locnlabel->setGeometry(QRect(30, 140, 331, 16));
        label = new QLabel(centralWidget);
        label->setObjectName(QString::fromUtf8("label"));
        label->setGeometry(QRect(40, 30, 101, 16));
        label_2 = new QLabel(centralWidget);
        label_2->setObjectName(QString::fromUtf8("label_2"));
        label_2->setGeometry(QRect(210, 30, 101, 16));
        startPushButton = new QPushButton(centralWidget);
        startPushButton->setObjectName(QString::fromUtf8("startPushButton"));
        startPushButton->setGeometry(QRect(30, 70, 141, 51));
        stopPushButton = new QPushButton(centralWidget);
        stopPushButton->setObjectName(QString::fromUtf8("stopPushButton"));
        stopPushButton->setGeometry(QRect(190, 70, 141, 51));
        label_3 = new QLabel(centralWidget);
        label_3->setObjectName(QString::fromUtf8("label_3"));
        label_3->setGeometry(QRect(40, 0, 331, 21));
        QFont font;
        font.setPointSize(18);
        label_3->setFont(font);
        lcdNumber = new QLCDNumber(centralWidget);
        lcdNumber->setObjectName(QString::fromUtf8("lcdNumber"));
        lcdNumber->setGeometry(QRect(340, 80, 64, 23));
        label_4 = new QLabel(centralWidget);
        label_4->setObjectName(QString::fromUtf8("label_4"));
        label_4->setGeometry(QRect(340, 60, 62, 16));
        spinBox = new QDoubleSpinBox(centralWidget);
        spinBox->setObjectName(QString::fromUtf8("spinBox"));
        spinBox->setGeometry(QRect(140, 20, 62, 24));
        spinBox->setDecimals(1);
        spinBox->setMinimum(0.1);
        spinBox->setMaximum(5);
        spinBox->setSingleStep(0.1);
        spinBox->setValue(1);
        MainWindow->setCentralWidget(centralWidget);
        menuBar = new QMenuBar(MainWindow);
        menuBar->setObjectName(QString::fromUtf8("menuBar"));
        menuBar->setGeometry(QRect(0, 0, 422, 22));
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
        MainWindow->setWindowTitle(QApplication::translate("MainWindow", "State Vector History Creator", 0, QApplication::UnicodeUTF8));
        locnlabel->setText(QString());
        label->setText(QApplication::translate("MainWindow", "Update every", 0, QApplication::UnicodeUTF8));
        label_2->setText(QApplication::translate("MainWindow", "seconds", 0, QApplication::UnicodeUTF8));
        startPushButton->setText(QApplication::translate("MainWindow", "Start Updating ", 0, QApplication::UnicodeUTF8));
        stopPushButton->setText(QApplication::translate("MainWindow", "Stop Updating ", 0, QApplication::UnicodeUTF8));
        label_3->setText(QApplication::translate("MainWindow", "State Vector History Creator", 0, QApplication::UnicodeUTF8));
        label_4->setText(QApplication::translate("MainWindow", "Updates", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
