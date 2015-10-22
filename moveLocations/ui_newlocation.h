/********************************************************************************
** Form generated from reading UI file 'newlocation.ui'
**
** Created: Wed Jul 1 13:59:22 2015
**      by: Qt User Interface Compiler version 4.8.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_NEWLOCATION_H
#define UI_NEWLOCATION_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QCheckBox>
#include <QtGui/QComboBox>
#include <QtGui/QDialog>
#include <QtGui/QDialogButtonBox>
#include <QtGui/QHeaderView>
#include <QtGui/QLabel>
#include <QtGui/QLineEdit>

QT_BEGIN_NAMESPACE

class Ui_newLocation
{
public:
    QDialogButtonBox *buttonBox;
    QLabel *label;
    QComboBox *parentComboBox;
    QLabel *label_3;
    QCheckBox *validRobotCheckBox;
    QCheckBox *validPersonCheckBox;
    QComboBox *proxemicsCombo;
    QLabel *label_4;
    QLineEdit *nameLineEdit;

    void setupUi(QDialog *newLocation)
    {
        if (newLocation->objectName().isEmpty())
            newLocation->setObjectName(QString::fromUtf8("newLocation"));
        newLocation->resize(565, 300);
        buttonBox = new QDialogButtonBox(newLocation);
        buttonBox->setObjectName(QString::fromUtf8("buttonBox"));
        buttonBox->setGeometry(QRect(40, 250, 341, 32));
        buttonBox->setOrientation(Qt::Horizontal);
        buttonBox->setStandardButtons(QDialogButtonBox::Cancel|QDialogButtonBox::Ok);
        label = new QLabel(newLocation);
        label->setObjectName(QString::fromUtf8("label"));
        label->setGeometry(QRect(20, 50, 62, 16));
        parentComboBox = new QComboBox(newLocation);
        parentComboBox->setObjectName(QString::fromUtf8("parentComboBox"));
        parentComboBox->setGeometry(QRect(100, 80, 341, 25));
        label_3 = new QLabel(newLocation);
        label_3->setObjectName(QString::fromUtf8("label_3"));
        label_3->setGeometry(QRect(20, 80, 62, 16));
        validRobotCheckBox = new QCheckBox(newLocation);
        validRobotCheckBox->setObjectName(QString::fromUtf8("validRobotCheckBox"));
        validRobotCheckBox->setGeometry(QRect(100, 120, 261, 21));
        validPersonCheckBox = new QCheckBox(newLocation);
        validPersonCheckBox->setObjectName(QString::fromUtf8("validPersonCheckBox"));
        validPersonCheckBox->setGeometry(QRect(100, 150, 261, 21));
        proxemicsCombo = new QComboBox(newLocation);
        proxemicsCombo->setObjectName(QString::fromUtf8("proxemicsCombo"));
        proxemicsCombo->setGeometry(QRect(200, 200, 341, 25));
        label_4 = new QLabel(newLocation);
        label_4->setObjectName(QString::fromUtf8("label_4"));
        label_4->setGeometry(QRect(20, 200, 171, 16));
        nameLineEdit = new QLineEdit(newLocation);
        nameLineEdit->setObjectName(QString::fromUtf8("nameLineEdit"));
        nameLineEdit->setGeometry(QRect(100, 50, 351, 24));

        retranslateUi(newLocation);
        QObject::connect(buttonBox, SIGNAL(accepted()), newLocation, SLOT(accept()));
        QObject::connect(buttonBox, SIGNAL(rejected()), newLocation, SLOT(reject()));

        QMetaObject::connectSlotsByName(newLocation);
    } // setupUi

    void retranslateUi(QDialog *newLocation)
    {
        newLocation->setWindowTitle(QApplication::translate("newLocation", "Locations", 0, QApplication::UnicodeUTF8));
        label->setText(QApplication::translate("newLocation", "Name", 0, QApplication::UnicodeUTF8));
        label_3->setText(QApplication::translate("newLocation", "Parent", 0, QApplication::UnicodeUTF8));
        validRobotCheckBox->setText(QApplication::translate("newLocation", "Valid Robot Location", 0, QApplication::UnicodeUTF8));
        validPersonCheckBox->setText(QApplication::translate("newLocation", "Valid Person Location", 0, QApplication::UnicodeUTF8));
        label_4->setText(QApplication::translate("newLocation", "Closest Proxemic Locaton", 0, QApplication::UnicodeUTF8));
        nameLineEdit->setInputMask(QString());
    } // retranslateUi

};

namespace Ui {
    class newLocation: public Ui_newLocation {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_NEWLOCATION_H
