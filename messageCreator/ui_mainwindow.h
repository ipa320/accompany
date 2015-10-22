/********************************************************************************
** Form generated from reading UI file 'mainwindow.ui'
**
** Created: Fri Jan 17 12:53:14 2014
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
    QComboBox *messageComboBox;
    QPushButton *changePushButton;
    QLabel *locnLabel;
    QLineEdit *messageLineEdit;
    QLabel *label_2;
    QLabel *label_3;
    QLabel *label_4;
    QMenuBar *menuBar;
    QToolBar *mainToolBar;
    QStatusBar *statusBar;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QString::fromUtf8("MainWindow"));
        MainWindow->resize(711, 388);
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
        languageComboBox->setGeometry(QRect(70, 80, 271, 25));
        messageComboBox = new QComboBox(centralWidget);
        messageComboBox->setObjectName(QString::fromUtf8("messageComboBox"));
        messageComboBox->setGeometry(QRect(70, 160, 321, 25));
        changePushButton = new QPushButton(centralWidget);
        changePushButton->setObjectName(QString::fromUtf8("changePushButton"));
        changePushButton->setGeometry(QRect(250, 290, 161, 41));
        locnLabel = new QLabel(centralWidget);
        locnLabel->setObjectName(QString::fromUtf8("locnLabel"));
        locnLabel->setGeometry(QRect(330, 20, 361, 16));
        messageLineEdit = new QLineEdit(centralWidget);
        messageLineEdit->setObjectName(QString::fromUtf8("messageLineEdit"));
        messageLineEdit->setGeometry(QRect(70, 240, 561, 24));
        label_2 = new QLabel(centralWidget);
        label_2->setObjectName(QString::fromUtf8("label_2"));
        label_2->setGeometry(QRect(50, 50, 62, 16));
        label_3 = new QLabel(centralWidget);
        label_3->setObjectName(QString::fromUtf8("label_3"));
        label_3->setGeometry(QRect(50, 120, 301, 16));
        label_4 = new QLabel(centralWidget);
        label_4->setObjectName(QString::fromUtf8("label_4"));
        label_4->setGeometry(QRect(40, 210, 301, 16));
        MainWindow->setCentralWidget(centralWidget);
        menuBar = new QMenuBar(MainWindow);
        menuBar->setObjectName(QString::fromUtf8("menuBar"));
        menuBar->setGeometry(QRect(0, 0, 711, 22));
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
        MainWindow->setWindowTitle(QApplication::translate("MainWindow", "Message Creator", 0, QApplication::UnicodeUTF8));
        label->setText(QApplication::translate("MainWindow", "Message Creator", 0, QApplication::UnicodeUTF8));
        changePushButton->setText(QApplication::translate("MainWindow", "Add/Update message", 0, QApplication::UnicodeUTF8));
        locnLabel->setText(QApplication::translate("MainWindow", "TextLabel", 0, QApplication::UnicodeUTF8));
        label_2->setText(QApplication::translate("MainWindow", "Language", 0, QApplication::UnicodeUTF8));
        label_3->setText(QApplication::translate("MainWindow", "Message (drop down to see existing messages)", 0, QApplication::UnicodeUTF8));
        label_4->setText(QApplication::translate("MainWindow", "Enter new or changed message here", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
