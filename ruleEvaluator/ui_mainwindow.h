/********************************************************************************
** Form generated from reading UI file 'mainwindow.ui'
**
** Created: Thu Jul 17 11:12:53 2014
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
#include <QtGui/QHeaderView>
#include <QtGui/QLabel>
#include <QtGui/QMainWindow>
#include <QtGui/QMenuBar>
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
    QLabel *dbLabel;
    QTableWidget *tableWidget;
    QPushButton *startPushButton;
    QPushButton *stopPushButton;
    QMenuBar *menuBar;
    QToolBar *mainToolBar;
    QStatusBar *statusBar;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QString::fromUtf8("MainWindow"));
        MainWindow->resize(635, 724);
        centralWidget = new QWidget(MainWindow);
        centralWidget->setObjectName(QString::fromUtf8("centralWidget"));
        dbLabel = new QLabel(centralWidget);
        dbLabel->setObjectName(QString::fromUtf8("dbLabel"));
        dbLabel->setGeometry(QRect(40, 640, 601, 16));
        tableWidget = new QTableWidget(centralWidget);
        if (tableWidget->columnCount() < 2)
            tableWidget->setColumnCount(2);
        tableWidget->setObjectName(QString::fromUtf8("tableWidget"));
        tableWidget->setGeometry(QRect(70, 50, 471, 571));
        tableWidget->setColumnCount(2);
        tableWidget->horizontalHeader()->setCascadingSectionResizes(false);
        startPushButton = new QPushButton(centralWidget);
        startPushButton->setObjectName(QString::fromUtf8("startPushButton"));
        startPushButton->setGeometry(QRect(70, 20, 97, 25));
        stopPushButton = new QPushButton(centralWidget);
        stopPushButton->setObjectName(QString::fromUtf8("stopPushButton"));
        stopPushButton->setGeometry(QRect(180, 20, 97, 25));
        MainWindow->setCentralWidget(centralWidget);
        menuBar = new QMenuBar(MainWindow);
        menuBar->setObjectName(QString::fromUtf8("menuBar"));
        menuBar->setGeometry(QRect(0, 0, 635, 22));
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
        MainWindow->setWindowTitle(QApplication::translate("MainWindow", "RuleEvaluator", 0, QApplication::UnicodeUTF8));
        dbLabel->setText(QApplication::translate("MainWindow", "TextLabel", 0, QApplication::UnicodeUTF8));
        startPushButton->setText(QApplication::translate("MainWindow", "Start", 0, QApplication::UnicodeUTF8));
        stopPushButton->setText(QApplication::translate("MainWindow", "Stop", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
