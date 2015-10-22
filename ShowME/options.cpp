#include "options.h"
#include "ui_options.h"
#include <QMessageBox>
#include <QtSql>

double tim;

options::options(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::options)
{
    ui->setupUi(this);
}

options::~options()
{
    delete ui;
}

void options::on_buttonBox_accepted()
{

    tim=ui->doubleSpinBox->value();
    return;
}

void options::on_buttonBox_rejected()
{
    return;
}

double options::getTiming()
{
    return tim;
}

void options::setTiming(double v)
{
   ui->doubleSpinBox->setValue(v);
   tim=v;
}

void options::setNegativeExampleState(bool swc)
{
    ui->negativeCheckBox->setChecked(swc);
}

bool options::getNegativeExampleState()
{
    return ui->negativeCheckBox->checkState();
}

void options::on_clearPushButton_clicked()
{

    QMessageBox::StandardButton reply;

    reply = QMessageBox::question(this, "Clear Negatives", "Are you sure you want to delete all negative examples?",
                                    QMessageBox::Yes|QMessageBox::No);
      if (reply == QMessageBox::Yes)
      {
         QSqlQuery query;

         query.exec("DELETE FROM SensorVectorHistory where classification = 'undefined'");

      }

}


