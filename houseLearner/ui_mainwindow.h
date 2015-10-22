/********************************************************************************
** Form generated from reading UI file 'mainwindow.ui'
**
** Created: Wed Jun 18 15:48:42 2014
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
#include <QtGui/QDateTimeEdit>
#include <QtGui/QHeaderView>
#include <QtGui/QLabel>
#include <QtGui/QMainWindow>
#include <QtGui/QPushButton>
#include <QtGui/QStatusBar>
#include <QtGui/QToolBar>
#include <QtGui/QWidget>

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QWidget *centralWidget;
    QLabel *label;
    QDateTimeEdit *dateFrom;
    QDateTimeEdit *dateTo;
    QLabel *label_2;
    QLabel *locnLabel;
    QPushButton *clearPushButton;
    QPushButton *fillPushButton;
    QPushButton *inflatePushButton;
    QCheckBox *checkBox;
    QCheckBox *checkBox_2;
    QToolBar *mainToolBar;
    QStatusBar *statusBar;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QString::fromUtf8("MainWindow"));
        MainWindow->resize(916, 676);
        centralWidget = new QWidget(MainWindow);
        centralWidget->setObjectName(QString::fromUtf8("centralWidget"));
        label = new QLabel(centralWidget);
        label->setObjectName(QString::fromUtf8("label"));
        label->setGeometry(QRect(270, 30, 331, 31));
        QFont font;
        font.setPointSize(18);
        label->setFont(font);
        dateFrom = new QDateTimeEdit(centralWidget);
        dateFrom->setObjectName(QString::fromUtf8("dateFrom"));
        dateFrom->setGeometry(QRect(360, 110, 211, 31));
        dateTo = new QDateTimeEdit(centralWidget);
        dateTo->setObjectName(QString::fromUtf8("dateTo"));
        dateTo->setGeometry(QRect(600, 110, 211, 31));
        label_2 = new QLabel(centralWidget);
        label_2->setObjectName(QString::fromUtf8("label_2"));
        label_2->setGeometry(QRect(120, 120, 201, 16));
        locnLabel = new QLabel(centralWidget);
        locnLabel->setObjectName(QString::fromUtf8("locnLabel"));
        locnLabel->setGeometry(QRect(30, 610, 321, 16));
        clearPushButton = new QPushButton(centralWidget);
        clearPushButton->setObjectName(QString::fromUtf8("clearPushButton"));
        clearPushButton->setGeometry(QRect(360, 190, 201, 25));
        fillPushButton = new QPushButton(centralWidget);
        fillPushButton->setObjectName(QString::fromUtf8("fillPushButton"));
        fillPushButton->setGeometry(QRect(360, 230, 201, 25));
        inflatePushButton = new QPushButton(centralWidget);
        inflatePushButton->setObjectName(QString::fromUtf8("inflatePushButton"));
        inflatePushButton->setGeometry(QRect(330, 410, 211, 71));
        checkBox = new QCheckBox(centralWidget);
        checkBox->setObjectName(QString::fromUtf8("checkBox"));
        checkBox->setGeometry(QRect(630, 190, 181, 21));
        checkBox_2 = new QCheckBox(centralWidget);
        checkBox_2->setObjectName(QString::fromUtf8("checkBox_2"));
        checkBox_2->setGeometry(QRect(630, 240, 181, 21));
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
        MainWindow->setWindowTitle(QApplication::translate("MainWindow", "MainWindow", 0, QApplication::UnicodeUTF8));
        label->setText(QApplication::translate("MainWindow", "Robot House Behaviour Learner", 0, QApplication::UnicodeUTF8));
        label_2->setText(QApplication::translate("MainWindow", "Use Sensor History Between", 0, QApplication::UnicodeUTF8));
        locnLabel->setText(QApplication::translate("MainWindow", "TextLabel", 0, QApplication::UnicodeUTF8));
        clearPushButton->setText(QApplication::translate("MainWindow", "Clear Sensor State Table?", 0, QApplication::UnicodeUTF8));
        fillPushButton->setText(QApplication::translate("MainWindow", "Fill Sensor State Table?", 0, QApplication::UnicodeUTF8));
        inflatePushButton->setText(QApplication::translate("MainWindow", "Normalise (inflate) sensor log", 0, QApplication::UnicodeUTF8));
        checkBox->setText(QApplication::translate("MainWindow", "Use normalised table?", 0, QApplication::UnicodeUTF8));
        checkBox_2->setText(QApplication::translate("MainWindow", "BATCHNORM", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
