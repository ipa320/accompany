/********************************************************************************
** Form generated from reading UI file 'mainwindow.ui'
**
** Created: Wed Jun 4 15:19:05 2014
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
#include <QtGui/QLineEdit>
#include <QtGui/QListWidget>
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
    QLabel *label;
    QComboBox *languageComboBox;
    QPushButton *changePushButton;
    QLabel *locnLabel;
    QLineEdit *messageLineEdit;
    QLabel *label_2;
    QLabel *label_4;
    QPushButton *changePushButton_3;
    QPushButton *changePushButton_4;
    QListWidget *listWidget;
    QLabel *locnLabel_2;
    QLabel *label_5;
    QComboBox *sensorComboBox;
    QListWidget *sensorListWidget;
    QLabel *locnLabel_3;
    QPushButton *deletePushButton;
    QMenuBar *menuBar;
    QToolBar *mainToolBar;
    QStatusBar *statusBar;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QString::fromUtf8("MainWindow"));
        MainWindow->resize(1059, 388);
        centralWidget = new QWidget(MainWindow);
        centralWidget->setObjectName(QString::fromUtf8("centralWidget"));
        label = new QLabel(centralWidget);
        label->setObjectName(QString::fromUtf8("label"));
        label->setGeometry(QRect(50, 10, 171, 31));
        QFont font;
        font.setPointSize(16);
        label->setFont(font);
        languageComboBox = new QComboBox(centralWidget);
        languageComboBox->setObjectName(QString::fromUtf8("languageComboBox"));
        languageComboBox->setGeometry(QRect(70, 80, 371, 25));
        changePushButton = new QPushButton(centralWidget);
        changePushButton->setObjectName(QString::fromUtf8("changePushButton"));
        changePushButton->setGeometry(QRect(140, 240, 161, 41));
        locnLabel = new QLabel(centralWidget);
        locnLabel->setObjectName(QString::fromUtf8("locnLabel"));
        locnLabel->setGeometry(QRect(320, 20, 351, 20));
        messageLineEdit = new QLineEdit(centralWidget);
        messageLineEdit->setObjectName(QString::fromUtf8("messageLineEdit"));
        messageLineEdit->setGeometry(QRect(70, 180, 561, 24));
        label_2 = new QLabel(centralWidget);
        label_2->setObjectName(QString::fromUtf8("label_2"));
        label_2->setGeometry(QRect(50, 50, 62, 16));
        label_4 = new QLabel(centralWidget);
        label_4->setObjectName(QString::fromUtf8("label_4"));
        label_4->setGeometry(QRect(50, 140, 301, 16));
        changePushButton_3 = new QPushButton(centralWidget);
        changePushButton_3->setObjectName(QString::fromUtf8("changePushButton_3"));
        changePushButton_3->setGeometry(QRect(350, 240, 201, 41));
        changePushButton_4 = new QPushButton(centralWidget);
        changePushButton_4->setObjectName(QString::fromUtf8("changePushButton_4"));
        changePushButton_4->setGeometry(QRect(470, 70, 201, 41));
        listWidget = new QListWidget(centralWidget);
        listWidget->setObjectName(QString::fromUtf8("listWidget"));
        listWidget->setGeometry(QRect(710, 50, 311, 131));
        locnLabel_2 = new QLabel(centralWidget);
        locnLabel_2->setObjectName(QString::fromUtf8("locnLabel_2"));
        locnLabel_2->setGeometry(QRect(750, 30, 211, 20));
        label_5 = new QLabel(centralWidget);
        label_5->setObjectName(QString::fromUtf8("label_5"));
        label_5->setGeometry(QRect(10, 300, 251, 16));
        sensorComboBox = new QComboBox(centralWidget);
        sensorComboBox->setObjectName(QString::fromUtf8("sensorComboBox"));
        sensorComboBox->setGeometry(QRect(260, 300, 301, 25));
        sensorListWidget = new QListWidget(centralWidget);
        sensorListWidget->setObjectName(QString::fromUtf8("sensorListWidget"));
        sensorListWidget->setGeometry(QRect(710, 210, 311, 131));
        locnLabel_3 = new QLabel(centralWidget);
        locnLabel_3->setObjectName(QString::fromUtf8("locnLabel_3"));
        locnLabel_3->setGeometry(QRect(770, 190, 211, 20));
        deletePushButton = new QPushButton(centralWidget);
        deletePushButton->setObjectName(QString::fromUtf8("deletePushButton"));
        deletePushButton->setGeometry(QRect(570, 300, 111, 31));
        MainWindow->setCentralWidget(centralWidget);
        menuBar = new QMenuBar(MainWindow);
        menuBar->setObjectName(QString::fromUtf8("menuBar"));
        menuBar->setGeometry(QRect(0, 0, 1059, 22));
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
        MainWindow->setWindowTitle(QApplication::translate("MainWindow", "Copy Behaviour", 0, QApplication::UnicodeUTF8));
        label->setText(QApplication::translate("MainWindow", "Copy Behaviour", 0, QApplication::UnicodeUTF8));
        changePushButton->setText(QApplication::translate("MainWindow", "Copy", 0, QApplication::UnicodeUTF8));
        locnLabel->setText(QApplication::translate("MainWindow", "Behaviours calling this behaviour", 0, QApplication::UnicodeUTF8));
        label_2->setText(QApplication::translate("MainWindow", "Behaviour", 0, QApplication::UnicodeUTF8));
        label_4->setText(QApplication::translate("MainWindow", "Enter new behaviour name  here", 0, QApplication::UnicodeUTF8));
        changePushButton_3->setText(QApplication::translate("MainWindow", " Delete original", 0, QApplication::UnicodeUTF8));
        changePushButton_4->setText(QApplication::translate("MainWindow", "Refresh", 0, QApplication::UnicodeUTF8));
        locnLabel_2->setText(QApplication::translate("MainWindow", "Behaviours calling this behaviour", 0, QApplication::UnicodeUTF8));
        label_5->setText(QApplication::translate("MainWindow", "LIst behaviours using this logical sensor", 0, QApplication::UnicodeUTF8));
        locnLabel_3->setText(QApplication::translate("MainWindow", "Behaviours using this logical", 0, QApplication::UnicodeUTF8));
        deletePushButton->setText(QApplication::translate("MainWindow", "Delete", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
