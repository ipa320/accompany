/********************************************************************************
** Form generated from reading UI file 'options.ui'
**
** Created: Thu Jul 17 10:44:26 2014
**      by: Qt User Interface Compiler version 4.8.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_OPTIONS_H
#define UI_OPTIONS_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QCheckBox>
#include <QtGui/QDialog>
#include <QtGui/QDialogButtonBox>
#include <QtGui/QDoubleSpinBox>
#include <QtGui/QHeaderView>
#include <QtGui/QLabel>
#include <QtGui/QPushButton>

QT_BEGIN_NAMESPACE

class Ui_options
{
public:
    QDialogButtonBox *buttonBox;
    QDoubleSpinBox *doubleSpinBox;
    QLabel *label;
    QLabel *label_2;
    QCheckBox *negativeCheckBox;
    QPushButton *clearPushButton;

    void setupUi(QDialog *options)
    {
        if (options->objectName().isEmpty())
            options->setObjectName(QString::fromUtf8("options"));
        options->resize(464, 219);
        buttonBox = new QDialogButtonBox(options);
        buttonBox->setObjectName(QString::fromUtf8("buttonBox"));
        buttonBox->setGeometry(QRect(260, 170, 171, 32));
        buttonBox->setOrientation(Qt::Horizontal);
        buttonBox->setStandardButtons(QDialogButtonBox::Cancel|QDialogButtonBox::Ok);
        doubleSpinBox = new QDoubleSpinBox(options);
        doubleSpinBox->setObjectName(QString::fromUtf8("doubleSpinBox"));
        doubleSpinBox->setGeometry(QRect(170, 60, 91, 24));
        doubleSpinBox->setDecimals(1);
        doubleSpinBox->setMinimum(0.1);
        doubleSpinBox->setMaximum(1);
        doubleSpinBox->setSingleStep(0.1);
        label = new QLabel(options);
        label->setObjectName(QString::fromUtf8("label"));
        label->setGeometry(QRect(70, 60, 91, 16));
        label_2 = new QLabel(options);
        label_2->setObjectName(QString::fromUtf8("label_2"));
        label_2->setGeometry(QRect(270, 60, 62, 16));
        negativeCheckBox = new QCheckBox(options);
        negativeCheckBox->setObjectName(QString::fromUtf8("negativeCheckBox"));
        negativeCheckBox->setGeometry(QRect(70, 100, 231, 21));
        clearPushButton = new QPushButton(options);
        clearPushButton->setObjectName(QString::fromUtf8("clearPushButton"));
        clearPushButton->setGeometry(QRect(70, 140, 221, 25));

        retranslateUi(options);
        QObject::connect(buttonBox, SIGNAL(accepted()), options, SLOT(accept()));
        QObject::connect(buttonBox, SIGNAL(rejected()), options, SLOT(reject()));

        QMetaObject::connectSlotsByName(options);
    } // setupUi

    void retranslateUi(QDialog *options)
    {
        options->setWindowTitle(QApplication::translate("options", "Sensor Log Timer", 0, QApplication::UnicodeUTF8));
        label->setText(QApplication::translate("options", "Update every", 0, QApplication::UnicodeUTF8));
        label_2->setText(QApplication::translate("options", "seconds", 0, QApplication::UnicodeUTF8));
        negativeCheckBox->setText(QApplication::translate("options", "Create negative examples", 0, QApplication::UnicodeUTF8));
        clearPushButton->setText(QApplication::translate("options", "Clear Negative Examples", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class options: public Ui_options {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_OPTIONS_H
