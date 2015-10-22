/********************************************************************************
** Form generated from reading UI file 'mainwindow.ui'
**
** Created: Mon Jan 20 16:44:21 2014
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
#include <QtGui/QTableWidget>
#include <QtGui/QToolBar>
#include <QtGui/QWidget>

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QWidget *centralWidget;
    QLabel *locnLabel;
    QLabel *userLabel;
    QComboBox *APTextComboBox;
    QLabel *userLabel_2;
    QLabel *userLabel_3;
    QComboBox *APPhraseComboBox;
    QPushButton *APCreatePushButton;
    QLabel *userLabel_4;
    QPushButton *clearAPPushButton;
    QComboBox *APPredComboBox;
    QLabel *userLabel_5;
    QTableWidget *tableWidget;
    QPushButton *APDeletePushButton;
    QPushButton *APRefreshPushButton;
    QMenuBar *menuBar;
    QToolBar *mainToolBar;
    QStatusBar *statusBar;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QString::fromUtf8("MainWindow"));
        MainWindow->resize(840, 751);
        centralWidget = new QWidget(MainWindow);
        centralWidget->setObjectName(QString::fromUtf8("centralWidget"));
        locnLabel = new QLabel(centralWidget);
        locnLabel->setObjectName(QString::fromUtf8("locnLabel"));
        locnLabel->setGeometry(QRect(760, 260, 62, 16));
        userLabel = new QLabel(centralWidget);
        userLabel->setObjectName(QString::fromUtf8("userLabel"));
        userLabel->setGeometry(QRect(550, 260, 181, 21));
        APTextComboBox = new QComboBox(centralWidget);
        APTextComboBox->setObjectName(QString::fromUtf8("APTextComboBox"));
        APTextComboBox->setGeometry(QRect(60, 90, 421, 25));
        APTextComboBox->setEditable(false);
        userLabel_2 = new QLabel(centralWidget);
        userLabel_2->setObjectName(QString::fromUtf8("userLabel_2"));
        userLabel_2->setGeometry(QRect(60, 50, 181, 21));
        userLabel_3 = new QLabel(centralWidget);
        userLabel_3->setObjectName(QString::fromUtf8("userLabel_3"));
        userLabel_3->setGeometry(QRect(60, 140, 181, 21));
        APPhraseComboBox = new QComboBox(centralWidget);
        APPhraseComboBox->setObjectName(QString::fromUtf8("APPhraseComboBox"));
        APPhraseComboBox->setGeometry(QRect(60, 180, 421, 25));
        APPhraseComboBox->setEditable(false);
        APCreatePushButton = new QPushButton(centralWidget);
        APCreatePushButton->setObjectName(QString::fromUtf8("APCreatePushButton"));
        APCreatePushButton->setGeometry(QRect(590, 140, 231, 25));
        userLabel_4 = new QLabel(centralWidget);
        userLabel_4->setObjectName(QString::fromUtf8("userLabel_4"));
        userLabel_4->setGeometry(QRect(60, 10, 251, 21));
        QFont font;
        font.setPointSize(16);
        userLabel_4->setFont(font);
        clearAPPushButton = new QPushButton(centralWidget);
        clearAPPushButton->setObjectName(QString::fromUtf8("clearAPPushButton"));
        clearAPPushButton->setGeometry(QRect(590, 90, 231, 25));
        APPredComboBox = new QComboBox(centralWidget);
        APPredComboBox->setObjectName(QString::fromUtf8("APPredComboBox"));
        APPredComboBox->setGeometry(QRect(60, 260, 421, 25));
        APPredComboBox->setEditable(false);
        userLabel_5 = new QLabel(centralWidget);
        userLabel_5->setObjectName(QString::fromUtf8("userLabel_5"));
        userLabel_5->setGeometry(QRect(60, 230, 211, 21));
        tableWidget = new QTableWidget(centralWidget);
        if (tableWidget->columnCount() < 6)
            tableWidget->setColumnCount(6);
        tableWidget->setObjectName(QString::fromUtf8("tableWidget"));
        tableWidget->setGeometry(QRect(60, 320, 741, 391));
        tableWidget->setColumnCount(6);
        APDeletePushButton = new QPushButton(centralWidget);
        APDeletePushButton->setObjectName(QString::fromUtf8("APDeletePushButton"));
        APDeletePushButton->setGeometry(QRect(590, 190, 231, 25));
        APRefreshPushButton = new QPushButton(centralWidget);
        APRefreshPushButton->setObjectName(QString::fromUtf8("APRefreshPushButton"));
        APRefreshPushButton->setGeometry(QRect(590, 40, 231, 25));
        MainWindow->setCentralWidget(centralWidget);
        menuBar = new QMenuBar(MainWindow);
        menuBar->setObjectName(QString::fromUtf8("menuBar"));
        menuBar->setGeometry(QRect(0, 0, 840, 22));
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
        MainWindow->setWindowTitle(QApplication::translate("MainWindow", "Action Possibility Creator", 0, QApplication::UnicodeUTF8));
        locnLabel->setText(QApplication::translate("MainWindow", "TextLabel", 0, QApplication::UnicodeUTF8));
        userLabel->setText(QApplication::translate("MainWindow", "TextLabel", 0, QApplication::UnicodeUTF8));
        userLabel_2->setText(QApplication::translate("MainWindow", "AP Text", 0, QApplication::UnicodeUTF8));
        userLabel_3->setText(QApplication::translate("MainWindow", "AP Phrase", 0, QApplication::UnicodeUTF8));
        APCreatePushButton->setText(QApplication::translate("MainWindow", "Create this Action Possibility", 0, QApplication::UnicodeUTF8));
        userLabel_4->setText(QApplication::translate("MainWindow", "Action Possibility Creator", 0, QApplication::UnicodeUTF8));
        clearAPPushButton->setText(QApplication::translate("MainWindow", "Set All AP Thresholds to Zero", 0, QApplication::UnicodeUTF8));
        userLabel_5->setText(QApplication::translate("MainWindow", "Fire This  Predictate If AP Chosen", 0, QApplication::UnicodeUTF8));
        APDeletePushButton->setText(QApplication::translate("MainWindow", "Delete an Action Possibility", 0, QApplication::UnicodeUTF8));
        APRefreshPushButton->setText(QApplication::translate("MainWindow", "Refresh", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
