/********************************************************************************
** Form generated from reading UI file 'timing.ui'
**
** Created: Fri Jun 27 11:40:16 2014
**      by: Qt User Interface Compiler version 4.8.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_TIMING_H
#define UI_TIMING_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QDialog>
#include <QtGui/QDialogButtonBox>
#include <QtGui/QDoubleSpinBox>
#include <QtGui/QHeaderView>
#include <QtGui/QLabel>

QT_BEGIN_NAMESPACE

class Ui_timing
{
public:
    QDialogButtonBox *buttonBox;
    QDoubleSpinBox *timerSpinBox;
    QLabel *label;
    QLabel *label_2;

    void setupUi(QDialog *timing)
    {
        if (timing->objectName().isEmpty())
            timing->setObjectName(QString::fromUtf8("timing"));
        timing->resize(609, 133);
        buttonBox = new QDialogButtonBox(timing);
        buttonBox->setObjectName(QString::fromUtf8("buttonBox"));
        buttonBox->setGeometry(QRect(440, 100, 141, 25));
        buttonBox->setStandardButtons(QDialogButtonBox::Apply|QDialogButtonBox::Cancel);
        timerSpinBox = new QDoubleSpinBox(timing);
        timerSpinBox->setObjectName(QString::fromUtf8("timerSpinBox"));
        timerSpinBox->setGeometry(QRect(270, 50, 101, 24));
        label = new QLabel(timing);
        label->setObjectName(QString::fromUtf8("label"));
        label->setGeometry(QRect(60, 50, 211, 16));
        label_2 = new QLabel(timing);
        label_2->setObjectName(QString::fromUtf8("label_2"));
        label_2->setGeometry(QRect(380, 50, 62, 16));

        retranslateUi(timing);

        QMetaObject::connectSlotsByName(timing);
    } // setupUi

    void retranslateUi(QDialog *timing)
    {
        timing->setWindowTitle(QApplication::translate("timing", "Dialog", 0, QApplication::UnicodeUTF8));
        label->setText(QApplication::translate("timing", "Capture sensor information every ", 0, QApplication::UnicodeUTF8));
        label_2->setText(QApplication::translate("timing", "seconds", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class timing: public Ui_timing {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_TIMING_H
