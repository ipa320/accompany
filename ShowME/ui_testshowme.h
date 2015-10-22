/********************************************************************************
** Form generated from reading UI file 'testshowme.ui'
**
** Created: Thu Jul 17 10:44:26 2014
**      by: Qt User Interface Compiler version 4.8.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_TESTSHOWME_H
#define UI_TESTSHOWME_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QDialog>
#include <QtGui/QHeaderView>
#include <QtGui/QTableWidget>

QT_BEGIN_NAMESPACE

class Ui_testShowMe
{
public:
    QTableWidget *showMeTableWidget;

    void setupUi(QDialog *testShowMe)
    {
        if (testShowMe->objectName().isEmpty())
            testShowMe->setObjectName(QString::fromUtf8("testShowMe"));
        testShowMe->resize(546, 511);
        showMeTableWidget = new QTableWidget(testShowMe);
        if (showMeTableWidget->columnCount() < 2)
            showMeTableWidget->setColumnCount(2);
        showMeTableWidget->setObjectName(QString::fromUtf8("showMeTableWidget"));
        showMeTableWidget->setGeometry(QRect(40, 50, 321, 431));
        QFont font;
        font.setPointSize(16);
        showMeTableWidget->setFont(font);
        showMeTableWidget->setColumnCount(2);

        retranslateUi(testShowMe);

        QMetaObject::connectSlotsByName(testShowMe);
    } // setupUi

    void retranslateUi(QDialog *testShowMe)
    {
        testShowMe->setWindowTitle(QApplication::translate("testShowMe", "ShowMe Test", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class testShowMe: public Ui_testShowMe {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_TESTSHOWME_H
