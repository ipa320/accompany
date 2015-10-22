/********************************************************************************
** Form generated from reading UI file 'mainwindow.ui'
**
** Created: Wed Mar 19 13:25:48 2014
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
    QComboBox *comboBox;
    QLabel *label;
    QLabel *locnLabel;
    QTableWidget *tableWidget;
    QCheckBox *checkBox;
    QCheckBox *checkBox_2;
    QPushButton *pushButton;
    QComboBox *scenComboBox;
    QLabel *label_2;
    QMenuBar *menuBar;
    QToolBar *mainToolBar;
    QStatusBar *statusBar;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QString::fromUtf8("MainWindow"));
        MainWindow->resize(1221, 554);
        centralWidget = new QWidget(MainWindow);
        centralWidget->setObjectName(QString::fromUtf8("centralWidget"));
        comboBox = new QComboBox(centralWidget);
        comboBox->setObjectName(QString::fromUtf8("comboBox"));
        comboBox->setGeometry(QRect(270, 20, 321, 25));
        label = new QLabel(centralWidget);
        label->setObjectName(QString::fromUtf8("label"));
        label->setGeometry(QRect(70, 30, 171, 16));
        locnLabel = new QLabel(centralWidget);
        locnLabel->setObjectName(QString::fromUtf8("locnLabel"));
        locnLabel->setGeometry(QRect(330, 470, 501, 20));
        tableWidget = new QTableWidget(centralWidget);
        if (tableWidget->columnCount() < 4)
            tableWidget->setColumnCount(4);
        tableWidget->setObjectName(QString::fromUtf8("tableWidget"));
        tableWidget->setGeometry(QRect(50, 80, 1171, 381));
        tableWidget->setEditTriggers(QAbstractItemView::AllEditTriggers);
        tableWidget->setHorizontalScrollMode(QAbstractItemView::ScrollPerPixel);
        tableWidget->setColumnCount(4);
        checkBox = new QCheckBox(centralWidget);
        checkBox->setObjectName(QString::fromUtf8("checkBox"));
        checkBox->setGeometry(QRect(650, 20, 131, 23));
        checkBox_2 = new QCheckBox(centralWidget);
        checkBox_2->setObjectName(QString::fromUtf8("checkBox_2"));
        checkBox_2->setGeometry(QRect(830, 20, 211, 23));
        pushButton = new QPushButton(centralWidget);
        pushButton->setObjectName(QString::fromUtf8("pushButton"));
        pushButton->setGeometry(QRect(270, 50, 97, 25));
        scenComboBox = new QComboBox(centralWidget);
        scenComboBox->setObjectName(QString::fromUtf8("scenComboBox"));
        scenComboBox->setGeometry(QRect(730, 50, 321, 25));
        label_2 = new QLabel(centralWidget);
        label_2->setObjectName(QString::fromUtf8("label_2"));
        label_2->setGeometry(QRect(660, 50, 62, 16));
        MainWindow->setCentralWidget(centralWidget);
        menuBar = new QMenuBar(MainWindow);
        menuBar->setObjectName(QString::fromUtf8("menuBar"));
        menuBar->setGeometry(QRect(0, 0, 1221, 22));
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
        label->setText(QApplication::translate("MainWindow", "Behaviour/Sequence", 0, QApplication::UnicodeUTF8));
        locnLabel->setText(QString());
        checkBox->setText(QApplication::translate("MainWindow", "Sort by priority", 0, QApplication::UnicodeUTF8));
        checkBox_2->setText(QApplication::translate("MainWindow", "Only Show Scheduled ", 0, QApplication::UnicodeUTF8));
        pushButton->setText(QApplication::translate("MainWindow", "Print", 0, QApplication::UnicodeUTF8));
        label_2->setText(QApplication::translate("MainWindow", "Scenario", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
