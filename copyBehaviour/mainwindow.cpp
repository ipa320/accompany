#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <QInputDialog>
#include <QDebug>
#include <QMessageBox>

QSqlDatabase db;
bool dbOpen;
QString experimentLocation;
MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    closeDownRequest = false;
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::setup()
{

    bool ok;
    QString host, user, pw, dBase;

    QFile file("../UHCore/Core/config.py");

    if (!file.exists())
    {
       qDebug()<<"No config.py found!!";
    }

    if (!file.open(QIODevice::ReadOnly | QIODevice::Text))
    {
        closeDownRequest = true;
        return;
    }

    QTextStream in(&file);
    while (!in.atEnd())
    {
       QString line = in.readLine();

       if (line.contains("mysql_log_user"))
       {
          user = line.section("'",3,3);
       }
       if (line.contains("mysql_log_password"))
       {
           pw = line.section("'",3,3);
       }
       if (line.contains("mysql_log_server"))
       {
          host = line.section("'",3,3);
       }
       if (line.contains("mysql_log_db"))
       {
          dBase = line.section("'",3,3);
       }
    }

    user = QInputDialog::getText ( this, "Accompany DB", "User:",QLineEdit::Normal,
                                     user, &ok);
    if (!ok)
    {
       closeDownRequest = true;
       return;
    }

    pw = QInputDialog::getText ( this, "Accompany DB", "Password:", QLineEdit::Password,
                                                                      pw, &ok);
    if (!ok)
    {
       closeDownRequest = true;
       return;
    }


    host = QInputDialog::getText ( this, "Accompany DB", "Host:",QLineEdit::Normal,
                                     host, &ok);
    if (!ok)
    {
      closeDownRequest = true;
      return;
    };

    dBase = QInputDialog::getText ( this, "Accompany DB", "Database:",QLineEdit::Normal,
                                     dBase, &ok);
    if (!ok)
    {
      closeDownRequest = true;
      return;
    };




    ui->locnLabel->setText(lv + ":" + user + ":" + host + ":" + dBase);


    db = QSqlDatabase::addDatabase("QMYSQL");

    db.setHostName(host);
    db.setDatabaseName(dBase);
    db.setUserName(user);
    db.setPassword(pw);

    dbOpen = db.open();

    if (!dbOpen)
    {

        QMessageBox msgBox;
        msgBox.setIcon(QMessageBox::Critical);

        msgBox.setText("Database error - login problem - see console log!");
        msgBox.exec();

        qCritical("Cannot open database: %s (%s)",
                  db.lastError().text().toLatin1().data(),
                  qt_error_string().toLocal8Bit().data());

        closeDownRequest = true;

        return;
    }
    else
    {
        qDebug() << "Database Opened";
    }

    // get experimental location


    QSqlQuery query("SELECT ExperimentalLocationId  FROM SessionControl WHERE SessionId = 1 LIMIT 1");

    if (query.next())
    {
       experimentLocation = query.value(0).toString();
    }
    else
    {
        QMessageBox msgBox;
        msgBox.setIcon(QMessageBox::Critical);

        msgBox.setText("Can find session control table!");
        msgBox.exec();
        closeDownRequest = true;
        return;
    }

    fillLanguageCombo();
    fillSensorCombo();


}

void MainWindow::fillLanguageCombo()
{
    QString seqQuery;
    QSqlQuery query;

    seqQuery = "SELECT name FROM Sequences where scenario != 'User Generated' and experimentalLocationId = " + experimentLocation + " order by name";

    query = seqQuery;

    query.exec();

    ui->languageComboBox->clear();

    while(query.next())
    {
        ui->languageComboBox->addItem(query.value(0).toString());
    }

    fillrelated();

}

void MainWindow::fillSensorCombo()
{
    QString seqQuery;
    QSqlQuery query;

    seqQuery = "SELECT sensorId,name,value, status FROM Sensors where sensorId > 499 and sensorId < 1000 order by sensorId";

    query = seqQuery;

    query.exec();

    ui->sensorComboBox->clear();

    while(query.next())
    {
        ui->sensorComboBox->addItem(query.value(0).toString() + "::" + query.value(1).toString() + "  (" + query.value(2).toString() +" " + query.value(3).toString() +")");
    }

    fillSensorRelated();



}

int MainWindow::fillrelated()
{
    QString seqQuery;
    QSqlQuery query;

    seqQuery  = "select name from ActionRules where ruleActionText like '%";
    seqQuery += ui->languageComboBox->currentText() + "%' and experimentalLocationId =" + experimentLocation + " order by name";

    query = seqQuery;

    qDebug()<<seqQuery;

    ui->listWidget->clear();

    query.exec();

    while(query.next())
   {
      ui->listWidget->addItem(query.value(0).toString());
    }

    return query.size();

}
int MainWindow::fillSensorRelated()
{
    QString seqQuery;
    QSqlQuery query;

    seqQuery  = "select distinct name from ActionRules where ruleActionText like '%";
    seqQuery += "::" + ui->sensorComboBox->currentText().section("::",0,0) + "::%' and experimentalLocationId =" + experimentLocation + " order by name";

    qDebug()<<seqQuery;

    query = seqQuery;

    qDebug()<<seqQuery;

    ui->sensorListWidget->clear();

    query.exec();

    while(query.next())
   {
      ui->sensorListWidget->addItem(query.value(0).toString());
    }

    return query.size();


}

void MainWindow::on_languageComboBox_currentIndexChanged(const QString &arg1)
{
    ui->messageLineEdit->setText(arg1);
    fillrelated();
}

void MainWindow::on_changePushButton_clicked()
{
    if (ui->messageLineEdit->text() == ui->languageComboBox->currentText())
    {
        QMessageBox msgBox;
        msgBox.setIcon(QMessageBox::Warning);

        msgBox.setText("You must give a different name to the new behaviour!");
        msgBox.exec();
        return;
    }

    if (ui->messageLineEdit->text().simplified() == "")
    {
        QMessageBox msgBox;
        msgBox.setIcon(QMessageBox::Warning);

        msgBox.setText("I need a name for the new behaviour!");
        msgBox.exec();
        return;
    }

    QString seqQuery;
    QSqlQuery query;
    QSqlQuery query1;

    seqQuery = "INSERT INTO Sequences ";
    seqQuery+= "SELECT '" + ui->messageLineEdit->text() + "',priority,interruptable,ruleCount,actionCount,schedulable,executable,scenario,scenarioDescription,experimentalLocationId FROM Sequences ";
    seqQuery+= "WHERE name = '" + ui->languageComboBox->currentText() + "' and experimentalLocationId = " + experimentLocation;

    qDebug()<<seqQuery;

    if (!query.exec(seqQuery))
    {
        QMessageBox msgBox;
        msgBox.setIcon(QMessageBox::Critical);

        msgBox.setText("Can't insert new behaviour - duplicate?");
        msgBox.exec();
        qDebug()<<query.lastError();
        return;
    }

    seqQuery = "SELECT * from ActionRules WHERE name = '" + ui->languageComboBox->currentText() + "' and experimentalLocationId = " + experimentLocation + " ORDER BY ruleOrder";

    qDebug()<<seqQuery;

    query = seqQuery;
/*
    if (!query.exec(seqQuery))
    {
        QMessageBox msgBox;
        msgBox.setIcon(QMessageBox::Critical);

        msgBox.setText("Can't select from ActionRules table!");
        msgBox.exec();
        qDebug()<<query.lastError();
        return;
    }
*/
    int ruleOrder = 0;

    query.exec();

    while (query.next())
    {
        seqQuery  = "INSERT INTO ActionRules  ";
        seqQuery += "VALUES ('" + ui->messageLineEdit->text() + "','";
        seqQuery +=  QString("%1").arg(ruleOrder) + "','";
        seqQuery +=  query.value(2).toString() + "','";
        seqQuery +=  query.value(3).toString() + "','";
        seqQuery +=  query.value(4).toString() + "'," +'"';

        seqQuery +=  query.value(5).toString() + '"'  + ",'";
        seqQuery +=  query.value(6).toString() + "'," + '"';
        seqQuery +=  query.value(7).toString() + '"' + ",'";
        seqQuery +=  query.value(8).toString() + "')";

        qDebug()<<seqQuery;

        query1.clear();

        if (!query1.exec(seqQuery))
        {
            QMessageBox msgBox;
            msgBox.setIcon(QMessageBox::Critical);

            msgBox.setText("Can't insert new behaviour rules into ActionRules");
            msgBox.exec();
            qDebug()<<query1.lastError();
            return;
        }

        ruleOrder++;

    }

    QMessageBox msgBox;
    msgBox.setIcon(QMessageBox::Information);

    msgBox.setText("Behaviour Successfully Added");
    msgBox.exec();

    on_changePushButton_4_clicked();


}


void MainWindow::on_changePushButton_3_clicked()
{

    if (fillrelated() > 0)
    {
        QMessageBox msgBox;
        msgBox.setIcon(QMessageBox::Warning);

        msgBox.setText("Can't delete because this behaviour is called by other behaviours!");
        msgBox.exec();
        return;
    }

    QString seqQuery;
    QSqlQuery query;

    seqQuery = "DELETE FROM Sequences ";
    seqQuery+= "WHERE name = '" + ui->languageComboBox->currentText() + "' and experimentalLocationId = " + experimentLocation;

    qDebug()<<seqQuery;

    if (!query.exec(seqQuery))
    {
        QMessageBox msgBox;
        msgBox.setIcon(QMessageBox::Critical);

        msgBox.setText("Can't delete behaviour from Sequences!");
        msgBox.exec();
        return;
    }

    seqQuery = "DELETE FROM ActionRules ";
    seqQuery+= "WHERE name = '" + ui->languageComboBox->currentText() + "' and experimentalLocationId = " + experimentLocation;

    qDebug()<<seqQuery;

    if (!query.exec(seqQuery))
    {
        QMessageBox msgBox;
        msgBox.setIcon(QMessageBox::Critical);

        msgBox.setText("Can't delete behaviour from ActionRules!");
        msgBox.exec();
        return;
    }

    QMessageBox msgBox;
    msgBox.setIcon(QMessageBox::Information);

    msgBox.setText("Behaviour Successfully Deleted");
    msgBox.exec();

    on_changePushButton_4_clicked();

}

void MainWindow::on_changePushButton_4_clicked()
{
    fillLanguageCombo();
}

void MainWindow::on_sensorComboBox_currentIndexChanged(const QString &arg1)
{
    fillSensorRelated();


}

void MainWindow::on_deletePushButton_clicked()
{
    if (fillSensorRelated() > 0)
    {
        QMessageBox msgBox;
        msgBox.setIcon(QMessageBox::Warning);

        msgBox.setText("Can't delete because this logical is called by other behaviours!");
        msgBox.exec();
        return;
    }

    QString seqQuery;
    QSqlQuery query;

    seqQuery = "DELETE FROM Sensors ";
    seqQuery+= "WHERE sensorId = '" + ui->sensorComboBox->currentText().section("::",0,0) + "'";

    qDebug()<<seqQuery;

    if (!query.exec(seqQuery))
    {
        QMessageBox msgBox;
        msgBox.setIcon(QMessageBox::Critical);

        msgBox.setText("Can't delete sensor!");
        msgBox.exec();
        return;
    }

    fillSensorCombo();
}
