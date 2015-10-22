#include "newlocation.h"
#include "ui_newlocation.h"

newLocation::newLocation(QSqlDatabase idb, QString istart, QString iend, QWidget *parent) :
    QDialog(parent),
    ui(new Ui::newLocation)
{
    ui->setupUi(this);

    setDB(idb);
    setValidLocns(istart,iend);

    qDebug()<<start<<end;

    QSqlQuery query;

    QString q = "SELECT locationId,name from Locations where locationId between " + start + " and " + end;

    qDebug()<<q;
    query.prepare(q);
    query.exec();

    while (query.next())
    {
        ui->parentComboBox->addItem(query.value(0).toString() + "::" + query.value(1).toString());
        ui->proxemicsCombo->addItem(query.value(0).toString() + "::" + query.value(1).toString());
    }

    ui->validRobotCheckBox->setChecked(true);
    ui->validPersonCheckBox->setChecked(true);

    ui->nameLineEdit->setText("New Location");

    q = "SELECT max(locationId) from Locations where locationId between " + start + " and " + end;

    nextId = 0;
    query.prepare(q);
    query.exec();
    if(query.next())
    {
        nextId = query.value(0).toInt() + 1;
    }

    qDebug()<<nextId;

}


QString newLocation::getName()
{
    return ui->nameLineEdit->text();
};

int  newLocation::getValidRobot()
{
    return ui->validRobotCheckBox->isChecked();

};

int  newLocation::getValidPerson()
{
    return ui->validPersonCheckBox->isChecked();

};

QString  newLocation::getParent()
{
    return ui->parentComboBox->currentText();

};

QString  newLocation::getNearest()
{
    return ui->proxemicsCombo->currentText();

};


newLocation::~newLocation()
{
    delete ui;
}
