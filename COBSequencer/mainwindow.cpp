#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QDebug>

#include <QSettings>
#include <QApplication>
#include <QProcess>
#include <QLineEdit>
#include <QInputDialog>
#include <QTableView>
#include <QMessageBox>
#include <QStringListModel>
#include <QStringList>
#include <QStandardItemModel>
#include <QFileDialog>

#include <QSqlRelationalTableModel>

#include <QApplication>

QSqlDatabase db;
bool dbOpen;

int ruleCount;
int actionCount;


int globalRuleCount;
int globalActionCount;

int andCount;

int experimentLocation;   // 1 = UH, 2=ZUYD, 3=Madopa
int defaultUserId;


MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    closeDownRequest = false;
}

void MainWindow::setup()
{
    bool ok;
    QString host, user, pw, dBase;

    user = QInputDialog::getText ( this, "Accompany DB", "User:",QLineEdit::Normal,
                                   "", &ok);
    if (!ok)
    {
       closeDownRequest = true;
       return;
    }



    pw = QInputDialog::getText ( this, "Accompany DB", "Password:", QLineEdit::Password,
                                                                    "", &ok);
    if (!ok)
    {
       closeDownRequest = true;
       return;
    }


    host = QInputDialog::getText ( this, "Accompany DB", "Host:",QLineEdit::Normal,
                                   "", &ok);
    if (!ok)
    {
       closeDownRequest = true;
       return;
    };

    dBase = QInputDialog::getText ( this, "Accompany DB", "Database:",QLineEdit::Normal,
                                   "", &ok);
    if (!ok)
    {
       closeDownRequest = true;
       return;
    };


    if (lv=="ZUYD")
    {
       if (host=="") host = "accompany1";
       if (user=="") user = "accompanyUser";
       if (pw=="") pw = "accompany";

    }
    else
    {
        if (host=="") host = "localhost";
        if (user=="") user = "rhUser";
        if (pw=="") pw = "waterloo";
    }


    if (dBase=="")  dBase = "Accompany";


    ui->userlabel->setText(lv + ":" + user + ":" + host);


    db = QSqlDatabase::addDatabase("QMYSQL");

    db.setHostName(host);
    db.setDatabaseName(dBase);
    db.setUserName(user);
    db.setPassword(pw);

    dbOpen = db.open();

    if (!dbOpen) {

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

    connect(&timer, SIGNAL(timeout()), this, SLOT(updateTime()));

    timer.start(1000);

    // get experimental location


    QSqlQuery query("SELECT ExperimentalLocationId, SessionUser FROM SessionControl WHERE SessionId = 1 LIMIT 1");

    if (query.next())
    {
       experimentLocation = query.value(0).toInt();
       defaultUserId = query.value(1).toInt();
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


    resetGui();

}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::changeEvent(QEvent *e)
{
    QMainWindow::changeEvent(e);
    switch (e->type()) {
    case QEvent::LanguageChange:
        ui->retranslateUi(this);
        break;
    default:
        break;
    }
}



void MainWindow::on_specUserLocationButton_clicked()
{

    ui->userLocationComboBox->setEnabled(true);
    ui->userLocationANDradioButton->setEnabled(true);
    ui->userLocationORradioButton->setEnabled(true);
    ui->userLocationUserComboBox->setEnabled(true);

    QString locQuery;

    locQuery = "SELECT  L1.locationId, L1.where, L2.where, L1.name, IF(STRCMP(L1.name,L2.name),L2.name,''), IF(STRCMP(L2.name,L3.name),L3.name,''), IF(STRCMP(L3.name,L4.name),L4.name,'')\
               FROM Locations L1,\
                    Locations L2,\
                    Locations L3,\
                    Locations L4\
               WHERE L2.locationId = L1.where\
                 AND L3.locationId = L2.where\
                 AND L4.locationId = L3.where\
                 AND L1.validUserLocation = 1";

   if (experimentLocation == 1)
   {
       locQuery += " AND (L1.locationId < 500 OR L1.locationId = 999) ORDER BY L1.locationId";
   }

    if (experimentLocation == 2)
    {
        locQuery += " AND ((L1.locationId > 599 AND L1.locationId < 700) or L1.locationId = 999) ORDER BY L1.locationId";
    }

    if (experimentLocation == 3)
    {
        locQuery += " AND ((L1.locationId > 699 AND L1.locationId < 800) or L1.locationId = 999) ORDER BY L1.locationId";
    }

//    qDebug() << locQuery;

    QSqlQuery query(locQuery);

    ui->userLocationComboBox->clear();

    QString q1, q2, q3;

    q1 = q2 = q3 = "";

    while(query.next())
    {
        q1 = q2 = q3 = "";

        if ( query.value(4).toString() != "")
        {
            if ( query.value(1) == 0)
            {
                q1 = "";
            }
            else
            {
                q1 = " in the " +   query.value(4).toString();
            }
        }

        if ( query.value(5).toString() != "")
        {
            if ( query.value(2) == 0)
            {
                q2 = "";
            }
            else
            {
                q2 = " of the " +   query.value(5).toString();
            }
        }

        ui->userLocationComboBox->addItem( "::" + query.value(0).toString() + ":: " + query.value(3).toString() + q1 + q2 );
    }




}


void MainWindow::on_AnyUserLocationButton_clicked()
{


    ui->userLocationComboBox->setEnabled(false);
    ui->userLocationANDradioButton->setEnabled(false);
    ui->userLocationORradioButton->setEnabled(false);
//    ui->userLocationUserComboBox->setEnabled(false);
//    ui->userLocationUserComboBox->clear();
    ui->userLocationComboBox->clear();


}

void MainWindow::on_robotLocationAnyButton_clicked()
{
    ui->robotLocationComboBox->setEnabled(false);
    ui->robotLocationANDradioButton->setEnabled(false);
    ui->robotLocationORradioButton->setEnabled(false);
    ui->robotLocationRobotComboBox->setEnabled(false);
    ui->robotLocationRobotComboBox->clear();
    ui->robotLocationComboBox->clear();
}
void MainWindow::on_robotLocationSpecButton_clicked()
{
    ui->robotLocationComboBox->setEnabled(true);
    ui->robotLocationANDradioButton->setEnabled(true);
    ui->robotLocationORradioButton->setEnabled(true);
    ui->robotLocationRobotComboBox->setEnabled(true);

    QString locQuery;
    locQuery = "SELECT  L1.locationId, L1.where, L2.where, L1.name, IF(STRCMP(L1.name,L2.name),L2.name,''), IF(STRCMP(L2.name,L3.name),L3.name,''), IF(STRCMP(L3.name,L4.name),L4.name,'')\
               FROM Locations L1,\
                    Locations L2,\
                    Locations L3,\
                    Locations L4\
               WHERE L2.locationId = L1.where\
                 AND L3.locationId = L2.where\
                 AND L4.locationId = L3.where\
                 AND L1.validRobotLocation = 1";

    if (experimentLocation == 1)
    {
        locQuery += " AND (L1.locationId < 500 OR L1.locationId = 999) ORDER BY L1.locationId";
    }

     if (experimentLocation == 2)
     {
         locQuery += " AND ((L1.locationId > 599 AND L1.locationId < 700) OR L1.locationId = 999) ORDER BY L1.locationId";
     }

     if (experimentLocation == 3)
     {
         locQuery += " AND ((L1.locationId > 699 AND L1.locationId < 800) OR L1.locationId = 999) ORDER BY L1.locationId";
     }
    QSqlQuery query(locQuery);

    ui->robotLocationComboBox->clear();

    QString q1, q2, q3;

    q1 = q2 = q3 = "";

    while(query.next())
    {
        q1 = q2 = q3 = "";

        if ( query.value(4).toString() != "")
        {
            if ( query.value(1).toInt() == 0)
            {
                q1 = "";
            }
            else
            {
                q1 = " in the " +   query.value(4).toString();
            }
        }

 

        if ( query.value(5).toString() != "")
        {
            if ( query.value(2).toInt() == 0)
            {
                q2 = "";
            }
            else
            {
                q2 = " of the " +   query.value(5).toString();
            }
        }

 

        ui->robotLocationComboBox->addItem( "::" + query.value(0).toString() + ":: " + query.value(3).toString() + q1 + q2 );
    }

    query.clear();

    query.prepare("SELECT robotId, robotName FROM Robot");

    query.exec();

    ui->robotLocationRobotComboBox->clear();

    while(query.next())
    {
        ui->robotLocationRobotComboBox->addItem("::"+ query.value(0).toString() + "::" + query.value(1).toString());
        ui->robotLocationRobotComboBox->setCurrentIndex(3);
    }


}






void MainWindow::on_AnyUserLocationButton_toggled(bool checked)
{


                if (checked)
                {
                   ruleCount--;
                }
                else
                {
                    ruleCount++;
                }



}

void MainWindow::on_robotLocationAnyButton_toggled(bool checked)
{


    if (checked)
    {
       ruleCount--;
    }
    else
    {
        ruleCount++;
    }


}



void MainWindow::on_IgnoreSensorsButton_clicked()
{
    ui->sensorTab->setEnabled(false);
    ui->diningSofa1CheckBox->setChecked(false);
    ui->diningSofa2CheckBox->setChecked(false);

}

void MainWindow::on_selectSensorsButton_clicked()
{
        ui->sensorTab->setEnabled(true);

        if (experimentLocation == 1)         // turn off HUYT and Madopa
        {
            ui->sensorTab->setTabEnabled(9,false);
            ui->sensorTab->setTabEnabled(8,false);
        }

        if (experimentLocation == 2)         // turn off UH and Madopa
        {
            ui->sensorTab->setTabEnabled(9,false);

            ui->sensorTab->setTabEnabled(0,false);
            ui->sensorTab->setTabEnabled(1,false);
            ui->sensorTab->setTabEnabled(2,false);
            ui->sensorTab->setTabEnabled(3,false);
            ui->sensorTab->setTabEnabled(4,false);
            ui->sensorTab->setTabEnabled(5,false);
            ui->sensorTab->setTabEnabled(6,false);
        }

        if (experimentLocation == 3)         // turn off UH and HUYT
        {
            ui->sensorTab->setTabEnabled(8,false);

            ui->sensorTab->setTabEnabled(0,false);
            ui->sensorTab->setTabEnabled(1,false);
            ui->sensorTab->setTabEnabled(2,false);
            ui->sensorTab->setTabEnabled(3,false);
            ui->sensorTab->setTabEnabled(4,false);
            ui->sensorTab->setTabEnabled(5,false);
            ui->sensorTab->setTabEnabled(6,false);
        }

}


void MainWindow::on_diningSofa1CheckBox_toggled(bool checked)
{

    ui->DRS1OccupiedRadioButton->setEnabled(checked);
    ui->DRS1NotOccupiedRadioButton->setEnabled(checked);
    ui->DRS1ANDradioButton->setEnabled(checked);
    ui->DRS1ORradioButton->setEnabled(checked);

    if (checked)
    {
       ruleCount++;
    }
    else
    {
        ruleCount--;
    }


}

void MainWindow::on_diningSofa2CheckBox_toggled(bool checked)
{

    ui->DRS2OccupiedRadioButton->setEnabled(checked);
    ui->DRS2NotOccupiedRadioButton->setEnabled(checked);
    ui->DRS2ANDradioButton->setEnabled(checked);
    ui->DRS2ORradioButton->setEnabled(checked);

    if (checked)
    {
       ruleCount++;
    }
    else
    {
        ruleCount--;
    }

}

void MainWindow::on_SeqComboBox_editTextChanged(QString )
{
    ui->prioritySpinBox->setEnabled(true);
    ui->InterruptcheckBox->setEnabled(true);
    ui->scheduleCheckBox->setEnabled(true);
    ui->seqAddButton->setEnabled(true);
    ui->seqDelButton->setEnabled(false);

    ui->seqDescLineEdit->setEnabled(true);
    ui->seqTypeComboBox->setEnabled(true);

}

void MainWindow::on_seqAddButton_clicked()
{
    QString str = ui->SeqComboBox->currentText();

    str = str.simplified();
    str.replace( " ", "" );

    if (str == "")
    {
        QMessageBox msgBox;
        msgBox.setIcon(QMessageBox::Critical);

        msgBox.setText("You need to give a name to this sequence!");
        msgBox.exec();
        return;
    }



    // for add's we reset the rule and action count to 0

    QSqlQuery query;

    query.prepare("INSERT INTO Sequences VALUES (:name, :priority, :inter, :ruleCount, :actionCount, :sched, :exec, :scen, :scendesc, :locn)");

    query.bindValue(":name",ui->SeqComboBox->currentText());
    query.bindValue(":priority",ui->prioritySpinBox->value());
    query.bindValue(":inter",ui->InterruptcheckBox->isChecked());
    query.bindValue(":sched",ui->scheduleCheckBox->isChecked());
    query.bindValue(":ruleCount",0);
    query.bindValue(":actionCount",0);
    query.bindValue(":exec",0);
    query.bindValue(":scen",ui->seqTypeComboBox->currentText());
    query.bindValue(":scendesc",ui->seqDescLineEdit->text());
    query.bindValue(":locn",experimentLocation);


    if (!query.exec())
    {

        qDebug() << query.lastQuery();

        QMessageBox msgBox;
        msgBox.setIcon(QMessageBox::Critical);

        msgBox.setText("Can't add/update Sequence table - duplicate?");
        msgBox.exec();

        qCritical("Cannot add/update: %s (%s)",
                  db.lastError().text().toLatin1().data(),
                  qt_error_string().toLocal8Bit().data());
        return;

    }


    ui->SeqComboBox->addItem(ui->SeqComboBox->currentText());

    ui->seqAddButton->setEnabled(false);
    ui->seqDelButton->setEnabled(true);

    ui->userLocationGroupBox->setEnabled(true);
    ui->robotLocationGroupBox->setEnabled(true);

    ui->sensorGroupBox->setEnabled(true);
    ui->timeGroupBox->setEnabled(true);
    ui->sensorActiveGroupBox->setEnabled(true);
    ui->lastActiveGroupBox->setEnabled(true);
    ui->actionGroupBox->setEnabled(true);


    ui->addRuleButton->setEnabled(true);
    ui->delRuleButton->setEnabled(false);

    ui->addActionButton->setEnabled(true);

 //   ui->pythonCreatePushButton->setEnabled(true);

    fillActionRuleTable("");

    globalRuleCount = 0;
    globalActionCount = 0;

}




void MainWindow::on_SeqComboBox_activated(QString seqName)
{
       ui->prioritySpinBox->setEnabled(true);
       ui->InterruptcheckBox->setEnabled(true);
       ui->scheduleCheckBox->setEnabled(true);
       ui->seqAddButton->setEnabled(false);

       ui->seqDelButton->setEnabled(true);

       ui->seqDescLineEdit->setEnabled(true);
       ui->seqTypeComboBox->setEnabled(true);


       QSqlQuery query;
       query.prepare("SELECT * FROM Sequences WHERE name = :name AND experimentalLocationId = :locn");
       query.bindValue(":locn",experimentLocation);
       query.bindValue(":name",seqName);

       if (query.exec())
       {
          while (query.next())
         {

              ui->prioritySpinBox->setValue(query.value(1).toInt());
              ui->InterruptcheckBox->setChecked(query.value(2).toInt());
              ui->scheduleCheckBox->setChecked(query.value(5).toInt());

              ui->seqDescLineEdit->setText(query.value(8).toString());

              ui->seqTypeComboBox->setCurrentIndex(ui->seqTypeComboBox->findText(query.value(7).toString()));


              globalRuleCount   = query.value(3).toInt();
              globalActionCount = query.value(4).toInt();


              fillActionRuleTable(seqName);
          }
      }
      else
      {
          QMessageBox msgBox;
          msgBox.setIcon(QMessageBox::Critical);

          msgBox.setText("Database error - can't select from Sequence table!");
          msgBox.exec();

          qCritical("Cannot select: %s (%s)",
                    db.lastError().text().toLatin1().data(),
                    qt_error_string().toLocal8Bit().data());
          return;
      }
     ui->userLocationGroupBox->setEnabled(true);
     ui->robotLocationGroupBox->setEnabled(true);

     ui->sensorGroupBox->setEnabled(true);
     ui->timeGroupBox->setEnabled(true);
     ui->sensorActiveGroupBox->setEnabled(true);
     ui->lastActiveGroupBox->setEnabled(true);

     ui->actionGroupBox->setEnabled(true);

}

void MainWindow::on_seqDelButton_clicked()
{

    int ret = QMessageBox::warning(this, tr("Care-O-Bot Sequencer"),
                                   tr("Do you really want to delete this sequence?"),
                                   QMessageBox::Cancel | QMessageBox::Yes);

    if (ret == QMessageBox::Cancel)
    {
        return;
    }


    QSqlQuery query;
    query.prepare("DELETE FROM Sequences WHERE name = :name AND experimentalLocationId = :locn");
    query.bindValue(":locn",experimentLocation);
    query.bindValue(":name",ui->SeqComboBox->currentText());

    if (!query.exec())
    {
        QMessageBox msgBox;
        msgBox.setIcon(QMessageBox::Critical);

        msgBox.setText("Database error - can't delete Sequence table!");
        msgBox.exec();

        qCritical("Cannot delete: %s (%s)",
                  db.lastError().text().toLatin1().data(),
                  qt_error_string().toLocal8Bit().data());
        return;

    }

    query.clear();

    query.prepare("DELETE FROM ActionRules WHERE name = :name AND experimentalLocationId = :locn");
    query.bindValue(":locn",experimentLocation);
    query.bindValue(":name",ui->SeqComboBox->currentText());

    if (!query.exec())
    {
        QMessageBox msgBox;
        msgBox.setIcon(QMessageBox::Critical);

        msgBox.setText("Database error - can't delete ActionRules table!");
        msgBox.exec();

        qCritical("Cannot delete: %s (%s)",
                  db.lastError().text().toLatin1().data(),
                  qt_error_string().toLocal8Bit().data());
        return;

    }

    query.clear();

    query.prepare("DELETE FROM userInterfaceGUI WHERE name = :name ");

    query.bindValue(":name",ui->SeqComboBox->currentText());

    if (!query.exec())
    {
        QMessageBox msgBox;
        msgBox.setIcon(QMessageBox::Critical);

        msgBox.setText("Database error - can't delete from userInterfaceGUI table!");
        msgBox.exec();

        qCritical("Cannot delete: %s (%s)",
                  db.lastError().text().toLatin1().data(),
                  qt_error_string().toLocal8Bit().data());
        return;

    }


    fillActionRuleTable("");

    ui->SeqComboBox->removeItem(ui->SeqComboBox->currentIndex());
    ui->prioritySpinBox->setValue(0);
    ui->InterruptcheckBox->setChecked(false);
    ui->scheduleCheckBox->setChecked(false);
    ui->SeqComboBox->clearEditText();

    ui->seqAddButton->setEnabled(false);
    ui->seqDelButton->setEnabled(false);
    ui->seqAddButton->setEnabled(false);

    ui->prioritySpinBox->setEnabled(false);
    ui->InterruptcheckBox->setEnabled(false);
    ui->scheduleCheckBox->setEnabled(false);

    ui->addRuleButton->setEnabled(false);
    ui->delRuleButton->setEnabled(false);

    ui->seqDescLineEdit->clear();

    ui->seqDescLineEdit->setEnabled(false);
    ui->seqTypeComboBox->setEnabled(false);

    ui->userLocationGroupBox->setEnabled(false);
    ui->robotLocationGroupBox->setEnabled(false);

    ui->sensorGroupBox->setEnabled(false);
    ui->timeGroupBox->setEnabled(false);
    ui->sensorActiveGroupBox->setEnabled(false);
    ui->lastActiveGroupBox->setEnabled(false);

    ui->actionGroupBox->setEnabled(false);
    ui->addActionButton->setEnabled(false);

    resetGui();

    globalRuleCount = 0;
    globalActionCount = 0;

}






void MainWindow::fillActionRuleTable(QString seqName)
{
    QString rc;
    rc.setNum(globalRuleCount);
    QString ac;
    ac.setNum(globalActionCount);

    qDebug()<<"--> Sequence:" + seqName + " " + rc + " " + ac;
    QSqlQuery query;
    query.prepare("SELECT * FROM ActionRules WHERE name = :name AND experimentalLocationId = :locn ORDER BY ruleOrder");
    query.bindValue(":locn",experimentLocation);

    query.bindValue(":name",seqName);

    QStringListModel *model = new QStringListModel();
    QStringList list;


    QString raText;

    if (query.exec())
    {
       while (query.next())
      {

           raText = "::";

           raText += query.value(1).toString();

           raText += "::  ";

           if (query.value(2) == "R")
           {
               raText += "RULE->" ;
           }

           if (query.value(2) == "A")
           {
               raText += "   ACTION->" ;
           }

           raText += query.value(5).toString();

           if (query.value(3).toBool())
           {
              raText += " NOT ";
           }

           if (query.value(4) == 1)
           {
              raText += " AND";
           }

           if (query.value(4) == 2)
           {
              raText += " OR";
           }

   //        qDebug() << raText;

           list << raText;
       }

   }

   model->setStringList(list);

   ui->ruleListView->setModel( model );
   ui->ruleListView->setEditTriggers(0);

   ui->addRuleButton->setEnabled(true);
   ui->delRuleButton->setEnabled(false);

   ui->addActionButton->setEnabled(true);
//   ui->pythonCreatePushButton->setEnabled(true);



}

void MainWindow::on_addRuleButton_clicked()
{
    // add new rules

    QString ruletext, rule, action;

    QString sequenceName = ui->SeqComboBox->currentText();
    QSqlQuery query;
    andCount = 0;


    if (ruleCount == 0)
    {
        QMessageBox msgBox;
        msgBox.setIcon(QMessageBox::Warning);

        msgBox.setText("No rule has been specified!");
        msgBox.exec();
        return;
    }


    if (ui->specUserLocationButton->isChecked())
    {

       ruletext = "Location of " + ui->userLocationUserComboBox->currentText() + " is " +  ui->userLocationComboBox->currentText();

    //   rule = "CALL checkUserLocation(" + ui->userLocationUserComboBox->currentText().section("::", 1, 1)
    //          + "," + ui->userLocationComboBox->currentText().section("::", 1, 1) + ")";

       QString QuestionLocationId = ui->userLocationComboBox->currentText().section("::", 1, 1);
       QString QuestionUserId     = ui->userLocationUserComboBox->currentText().section("::", 1, 1);

       rule = "SELECT locationId FROM Users WHERE (locationId IN (\
                 SELECT c.locationId FROM (\
                     SELECT b.* FROM Locations a, Locations b\
                     WHERE  a.locationId = " + QuestionLocationId +
                       " AND  a.locationId = b.where) b, Locations c\
                                WHERE b.locationId = c.where\
                     UNION \
                       SELECT b.locationId  FROM Locations a, Locations b \
                       WHERE  a.locationId = " + QuestionLocationId +
                     "    AND  a.locationId = b.where \
                     UNION \
                       SELECT b.locationId FROM Locations b\
                        WHERE  b.locationId =" + QuestionLocationId +")\
                        AND userId = " + QuestionUserId + ") or\
                        (" + QuestionLocationId + " = 0 and userId = " + QuestionUserId + ")";

       rule=rule.simplified();



       qDebug()<<rule;

       action = "";

       query.clear();
       query.prepare("INSERT INTO ActionRules (name, ruleOrder, ruleType, andOrConnector, ruleActionText, rule, action, experimentalLocationId)\
                     VALUES (:name, :ruleOrder, :ruleType, :andOrConnector, :ruleActionText, :rule, :action, :locn)");

       query.bindValue(":name",sequenceName );
       query.bindValue(":ruleOrder",globalRuleCount++);
       query.bindValue(":ruleType","R");
       query.bindValue(":andOrConnector",0);     // default to no connector
       query.bindValue(":ruleActionText",ruletext);
       query.bindValue(":rule",rule);
       query.bindValue(":actionText",action); //???
       query.bindValue(":locn",experimentLocation);


       if (ruleCount > 1)
       {
          if (andCount+1 < ruleCount)
          {
             if (ui->userLocationANDradioButton->isChecked())
             {
                query.bindValue(":andOrConnector",1);

             }

             if (ui->userLocationORradioButton->isChecked())
             {
                query.bindValue(":andOrConnector",2);
             }
             andCount++;
          }
        }

       if (!query.exec())
       {
           qDebug() << "Can't add user locn record to rules table!" << query.executedQuery();
       }

    }

    if (ui->robotLocationSpecButton->isChecked())
    {

        ruletext = "Location of " + ui->robotLocationRobotComboBox->currentText() + " is " +  ui->robotLocationComboBox->currentText();

   //     rule = "CALL checkRobotLocation(" +  ui->robotLocationRobotComboBox->currentText().section("::", 1, 1)
   //             + "," +  ui->robotLocationComboBox->currentText().section("::", 1, 1) + ")";

        QString QuestionLocationId = ui->robotLocationComboBox->currentText().section("::", 1, 1);
        QString QuestionRobotId    = ui->robotLocationRobotComboBox->currentText().section("::", 1, 1);
        QString QuestionUserId     = ui->userLocationUserComboBox->currentText().section("::", 1, 1);

        if (QuestionLocationId == "999")
        {
            rule = "SELECT r.locationId, u.locationId FROM Robot r, Users u WHERE u.userid = " + QuestionUserId + " AND r.robotId = " + QuestionRobotId + " AND r.locationId = u.locationId";
        }
        else
        {                                                                   // complex rule due to hierachy of locns
           rule = "SELECT locationId FROM Robot WHERE (locationId IN (\
                  SELECT c.locationId FROM (\
                      SELECT b.* FROM Locations a, Locations b\
                      WHERE  a.locationId = " + QuestionLocationId +
                        " AND  a.locationId = b.where) b, Locations c\
                                 WHERE b.locationId = c.where\
                      UNION \
                        SELECT b.locationId  FROM Locations a, Locations b \
                        WHERE  a.locationId = " + QuestionLocationId +
                      "    AND  a.locationId = b.where \
                      UNION \
                        SELECT b.locationId FROM Locations b\
                         WHERE  b.locationId =" + QuestionLocationId +")\
                         AND robotId = " + QuestionRobotId + ") or\
                         (" + QuestionLocationId + " = 0 and robotId = " + QuestionRobotId + ")";

            rule=rule.simplified();
          }

        action = "";

        query.clear();
        query.prepare("INSERT INTO ActionRules (name, ruleOrder, ruleType, andOrConnector, ruleActionText, rule, action, experimentalLocationId)\
                      VALUES (:name, :ruleOrder, :ruleType, :andOrConnector, :ruleActionText, :rule, :action, :locn)");

        query.bindValue(":name",sequenceName );
        query.bindValue(":ruleOrder",globalRuleCount++);
        query.bindValue(":ruleType","R");
        query.bindValue(":andOrConnector",0);     // default to no connector
        query.bindValue(":ruleActionText",ruletext);
        query.bindValue(":rule",rule);
        query.bindValue(":actionText",action);

        query.bindValue(":locn",experimentLocation);


        if (ruleCount > 1)
        {
           if (andCount+1 < ruleCount)
           {
              if (ui->robotLocationANDradioButton->isChecked())
              {
                 query.bindValue(":andOrConnector",1);
              }
              if (ui->robotLocationORradioButton->isChecked())
              {
                 query.bindValue(":andOrConnector",2);
              }
              andCount++;
           }
        }

        if (!query.exec())
        {
            qDebug() << "Can't add robot locn record to rules table!" << query.executedQuery();
        }
    }


    //----------------------------------------------------------------


    // NOTE: pressure mat  return 0 if occupied and 1 if not occupied!!!


    fillRuleActionTable("Dining room seat 1",
                        20,
                       "unoccupied:occupied",
                       ui->diningSofa1CheckBox->isChecked(),
                       !ui->DRS1OccupiedRadioButton->isChecked(),
                       ui->DRS1ANDradioButton->isChecked(),
                       ui->DRS1ORradioButton->isChecked());

    fillRuleActionTable("Dining room seat 2",
                        21,
                       "unoccupied:occupied",
                       ui->diningSofa2CheckBox->isChecked(),
                       !ui->DRS2OccupiedRadioButton->isChecked(),
                       ui->DRS2ANDradioButton->isChecked(),
                       ui->DRS2ORradioButton->isChecked());

    fillRuleActionTable("Big Cupboard Bottom Drawer",
                        24,
                        "open:closed",
                       ui->BigCupboardDoorBottomCheckBox->isChecked(),
                       ui->BCDBOpenRadioButton->isChecked(),
                       ui->BCDBANDRadioButton->isChecked(),
                       ui->BCDBORRadioButton->isChecked());

    fillRuleActionTable("Big Cupboard Top Drawer",
                        25,
                        "open:closed",
                       ui->BigCupboardDoorTopCheckBox->isChecked(),
                       ui->BCDTOpenRadioButton->isChecked(),
                       ui->BCDTANDRadioButton->isChecked(),
                       ui->BCDTORRadioButton->isChecked());

    fillRuleActionTable("Small Cupboard Left Drawer",
                        26,
                        "open:closed",
                       ui->SmallCupboardDoorLeftCheckBox->isChecked(),
                       ui->SCDLOpenRadioButton->isChecked(),
                       ui->SCDLANDRadioButton->isChecked(),
                       ui->SCDLORRadioButton->isChecked());

    fillRuleActionTable("Small Cupboard Right Drawer",
                        27,
                        "open:closed",
                       ui->SmallCupboardDooRightCheckBox->isChecked(),
                       ui->SCDROpenRadioButton->isChecked(),
                       ui->SCDRANDRadioButton->isChecked(),
                       ui->SCDRORRadioButton->isChecked());

    fillRuleActionTable("Small Cupboard Bottom Drawer",
                        28,
                        "open:closed",
                       ui->SmallCupboardDrawBottomCheckBox->isChecked(),
                       ui->SCDBOpenRadioButton->isChecked(),
                       ui->SCDBANDRadioButton->isChecked(),
                       ui->SCDBORRadioButton->isChecked());

    fillRuleActionTable("Small Cupboard Middle Drawer",
                        29,
                        "open:closed",
                       ui->SmallCupboardDoorMiddleCheckBox->isChecked(),
                       ui->SCDMOpenRadioButton->isChecked(),
                       ui->SCDMANDRadioButton->isChecked(),
                       ui->SCDMORRadioButton->isChecked());

    fillRuleActionTable("Small Cupboard Top Drawer",
                        30,
                        "open:closed",
                       ui->SmallCupboardDoorTopCheckBox->isChecked(),
                       ui->SCDTOpenRadioButton->isChecked(),
                       ui->SCDTANDRadioButton->isChecked(),
                       ui->SCDTORRadioButton->isChecked());


    // NOTE: pressure mat  return 0 if occupied and 1 if not occupied!!!

    fillRuleActionTable("Living room sofa seat 1",
                        15,
                       "unoccupied:occupied",
                       ui->livingSofa1CheckBox->isChecked(),
                       !ui->LRS1OccupiedRadioButton->isChecked(),
                       ui->LRS1ANDRadioButton->isChecked(),
                       ui->LRS1ORRadioButton->isChecked());


      fillRuleActionTable("Living room sofa seat 2",
                          16,
                         "unoccupied:occupied",
                         ui->livingSofa2CheckBox->isChecked(),
                         !ui->LRS2OccupiedRadioButton->isChecked(),
                         ui->LRS2ANDRadioButton->isChecked(),
                         ui->LRS2ORRadioButton->isChecked());

      fillRuleActionTable("Living room sofa seat 3",
                          17,
                         "unoccupied:occupied",
                         ui->livingSofa3CheckBox->isChecked(),
                         !ui->LRS3OccupiedRadioButton->isChecked(),
                         ui->LRS3ANDRadioButton->isChecked(),
                         ui->LRS3ORRadioButton->isChecked());

      fillRuleActionTable("Living room sofa seat 4",
                        18,
                        "unoccupied:occupied",
                        ui->livingSofa4CheckBox->isChecked(),
                        !ui->LRS4OccupiedRadioButton->isChecked(),
                        ui->LRS4ANDRadioButton->isChecked(),
                        ui->LRS4ORRadioButton->isChecked());

       fillRuleActionTable("Living room sofa seat 5",
                           19,
                           "unoccupied:occupied",
                           ui->livingSofa5CheckBox->isChecked(),
                           !ui->LRS5OccupiedRadioButton->isChecked(),
                           ui->LRS5ANDRadioButton->isChecked(),
                           ui->LRS5ORRadioButton->isChecked());

       fillRuleActionTable("Television",
                          49,
                         "Wattage",
                          ui->TVcheckBox->isChecked(),
                          ui->TVSpinBox->value(),
                          ui->TVANDRadioButton->isChecked(),
                          ui->TVORRadioButton->isChecked());


        fillRuleActionTable("Kitchen Hot Tap",
                            1,
                           "Temperature",
                          ui->HotTapKitchenCheckBox->isChecked(),
                          ui->HTKTempSpinBox->value(),
                          ui->HTKANDRadioButton->isChecked(),
                          ui->HTKORRadioButton->isChecked());

        fillRuleActionTable("Kitchen Cold Tap",
                            2,
                            "Temperature",
                            ui->ColdTapKitchenCheckBox->isChecked(),
                            ui->CTKTempSpinBox->value(),
                            ui->CTKANDRadioButton->isChecked(),
                            ui->CTKORRadioButton->isChecked());


        fillRuleActionTable("Kitchen Ceiling Left Door",
                            3,
                            "open:closed",
                            ui->KitchenCeilDoorLeftCheckBox->isChecked(),
                            ui->KCDLOpenRadioButton->isChecked(),
                            ui->KCDLANDRadioButton->isChecked(),
                            ui->KCDLORRadioButton->isChecked());

         fillRuleActionTable("Kitchen Ceiling Middle Door",
                            4,
                            "open:closed",
                            ui->KitchenCeilDoorMiddleCheckBox->isChecked(),
                            ui->KCDMOpenRadioButton->isChecked(),
                            ui->KCDMANDRadioButton->isChecked(),
                            ui->KCDMORRadioButton->isChecked());


          fillRuleActionTable("Kitchen Ceiling Right Door",
                              5,
                              "open:closed",
                              ui->KitchenCeilDoorRightCheckBox->isChecked(),
                              ui->KCDROpenRadioButton->isChecked(),
                              ui->KCDRANDRadioButton->isChecked(),
                              ui->KCDRORRadioButton->isChecked());

          fillRuleActionTable("Kitchen Floor Middle Drawer",
                               6,
                               "open:closed",
                               ui->KitchenFloorDrawerMiddleCheckBox->isChecked(),
                               ui->KFDMOpenRadioButton->isChecked(),
                               ui->KFDMANDRadioButton->isChecked(),
                               ui->KFDMORRadioButton->isChecked());

          fillRuleActionTable("Kitchen Floor Right Drawer",
                               7,
                               "open:closed",
                               ui->KitchenFloorDrawerRightCheckBox->isChecked(),
                               ui->KFDROpenRadioButton->isChecked(),
                               ui->KFDRANDRadioButton->isChecked(),
                               ui->KFDRORRadioButton->isChecked());


           fillRuleActionTable("Kitchen Floor Left Door",
                                10,
                                "open:closed",
                                ui->KitchenFloorDoorLeftCheckBox->isChecked(),
                                ui->KFDoorLOpenRadioButton->isChecked(),
                                ui->KFDoorLANDRadioButton->isChecked(),
                                ui->KFDoorLORRadioButton->isChecked());

            fillRuleActionTable("Kitchen Floor Middle Door",
                                8,
                                "open:closed",
                                ui->KitchenFloorDoorMiddleCheckBox->isChecked(),
                                ui->KFDoorMOpenRadioButton->isChecked(),
                                ui->KFDoorMANDRadioButton->isChecked(),
                                ui->KFDoorMORRadioButton->isChecked());

            fillRuleActionTable("Kitchen Floor Right Door",
                                9,
                                "open:closed",
                                ui->KitchenFloorDoorRightCheckBox->isChecked(),
                                ui->KFDoorROpenRadioButton->isChecked(),
                                ui->KFDoorRANDRadioButton->isChecked(),
                                ui->KFDoorRORRadioButton->isChecked());


           fillRuleActionTable("Microwave",
                               54,
                               "Wattage",
                               ui->microwaveCheckBox->isChecked(),
                               ui->microwaveSpinBox->value(),
                               ui->microwaveANDRadioButton->isChecked(),
                               ui->microwaveORRadioButton->isChecked());

           fillRuleActionTable("Cooker",
                               44,
                               "Wattage",
                               ui->cookerCheckBox->isChecked(),
                               ui->cookerSpinBox->value(),
                               ui->cookerANDRadioButton->isChecked(),
                               ui->cookerORRadioButton->isChecked());

           fillRuleActionTable("Fridge Freezer",
                               50,
                               "Wattage",
                               ui->fridgeCheckBox->isChecked(),
                               ui->fridgeSpinBox->value(),
                               ui->fridgeANDRadioButton->isChecked(),
                               ui->fridgeORRadioButton->isChecked());

           fillRuleActionTable("Kettle",
                               51,
                               "On:Off",
                               ui->kettleCheckBox->isChecked(),
                               ui->KettleOnRadioButton->isChecked(),
                               ui->kettleANDRadioButton->isChecked(),
                               ui->kettleORRadioButton->isChecked());

           fillRuleActionTable("Toaster",
                               56,
                               "Wattage",
                               ui->toasterCheckBox->isChecked(),
                               ui->toasterSpinBox->value(),
                               ui->toasterANDRadioButton->isChecked(),
                               ui->toasterORRadioButton->isChecked());

           fillRuleActionTable("Dishwasher",
                               55,
                               "Wattage",
                               ui->dishwasherCheckBox->isChecked(),
                               ui->dishwasherSpinBox->value(),
                               ui->dishwasherANDRadioButton->isChecked(),
                               ui->dishwasherORRadioButton->isChecked());

           fillRuleActionTable("Doorbell",
                               59,
                               "Wattage",
                               ui->doorbellCheckBox->isChecked(),
                               ui->doorbellSpinBox->value(),
                               ui->doorbellANDRadioButton->isChecked(),
                               ui->doorbellORRadioButton->isChecked());

    // ZUYD Sensors

           fillRuleActionTable("ZUYD Sofa seat 1",
                               301,
                              "occupied:unoccupied",
                              ui->HUYTlivingSofa1CheckBox->isChecked(),
                              ui->HUYTLRS1OccupiedRadioButton->isChecked(),
                              ui->HUYTLRS1ANDRadioButton->isChecked(),
                              ui->HUYTLRS1ORRadioButton->isChecked());

           fillRuleActionTable("ZUYD Sofa seat 2",
                               304,
                              "occupied:unoccupied",
                              ui->HUYTlivingSofa2CheckBox->isChecked(),
                              ui->HUYTLRS2OccupiedRadioButton->isChecked(),
                              ui->HUYTLRS2ANDRadioButton->isChecked(),
                              ui->HUYTLRS2ORRadioButton->isChecked());

           fillRuleActionTable("ZUYD Small Sofa",
                               305,
                              "occupied:unoccupied",
                              ui->HUYTlivingSofa3CheckBox->isChecked(),
                              ui->HUYTLRS3OccupiedRadioButton->isChecked(),
                              ui->HUYTLRS3ANDRadioButton->isChecked(),
                              ui->HUYTLRS3ORRadioButton->isChecked());


           fillRuleActionTable("ZUYD Cup",
                               307,
                               "Full:Empty",
                               ui->cupLevelCheckBox->isChecked(),
                               ui->HUYTCupFullRadioButton->isChecked(),
                               ui->cupLevelANDRadioButton->isChecked(),
                               ui->cupLevelORRadioButton->isChecked());

           fillRuleActionTable("ZUYD Doorbell",
                               302,
                               "On:Off",
                               ui->ZUYDDoorbellCheckBox->isChecked(),
                               ui->ZUYDDoorbellOnRadioButton->isChecked(),
                               ui->ZUYDDoorbellANDRadioButton->isChecked(),
                               ui->ZUYDDoorbellORRadioButton->isChecked());

           fillRuleActionTable("ZUYD Fridge Door",             // 1 is closed, 0 is open
                               303,
                               "open:closed",
                               ui->ZUYDFridgeCheckBox->isChecked(),
                               ui->ZUYDFridgeOnRadioButton->isChecked(),
                               ui->ZUYDFridgeANDRadioButton->isChecked(),
                               ui->ZUYDFridgeORRadioButton->isChecked());


    //---------------------------------------------------------------

    // Goals

        if (ui->Goal1ComboBox->currentText().section("::", 1, 1).toInt() > 899)  // contexts
        {
            fillRuleActionTable(ui->Goal1ComboBox->currentText(),
                            ui->Goal1ComboBox->currentText().section("::", 1, 1).toInt(),
                            "True:False",
                            ui->Goal1CheckBox->isChecked(),
                            ui->Goal1TRUERadioButton->isChecked(),
                            ui->Goal1ANDRadioButton->isChecked(),
                            ui->Goal1ORRadioButton->isChecked());
        }
        else   // goals/conditions
        {
            if (ui->Goal1CheckBox->isChecked())
            {
              QString upper = ui->Goal1LineEdit->text();
              upper[0] = upper.at(0).toTitleCase();
              ui->Goal1LineEdit->setText(upper);
            }
            fillRuleActionTable(ui->Goal1ComboBox->currentText(),
                            ui->Goal1ComboBox->currentText().section("::", 1, 1).toInt(),
                            ui->Goal1LineEdit->text(),
                            ui->Goal1CheckBox->isChecked(),
                            true,
                            ui->Goal1ANDRadioButton->isChecked(),
                            ui->Goal1ORRadioButton->isChecked());


        }

    //-------------------------------------------------------

    if (ui->timeCheckBox->isChecked())
    {
        if (ui->timeAtRadioButton->isChecked())
        {
            QString v = ui->timeEdit_1->time().toString();

            ruletext = "Time is on or after " + v;

         //   rule = "SELECT 1 from DUAL Where (SELECT TIMEDIFF(CURTIME(),'" + v + "') > 0)";

            rule = "CALL spBetweenTimeCheck('" + v + "','23:59:59')";
        }


        if (ui->timeBetweenRadioButton->isChecked())
        {
            if (ui->timeEdit_2->time() <= ui->timeEdit_1->time())
            {
                QMessageBox msgBox;
                msgBox.setIcon(QMessageBox::Warning);

                msgBox.setText("The second time must be after the first time!");
                msgBox.exec();
                ui->timeEdit_2->setFocus();
                return;
            }

            QString v1 = ui->timeEdit_1->time().toString();
            QString v2 = ui->timeEdit_2->time().toString();

            ruletext = "Time is between " + v1 + " and " + v2;

      /*      rule = "SELECT 1 from DUAL Where (SELECT TIMEDIFF(CURTIME(),'" + v1 +\
                           "') > 0) AND (SELECT TIMEDIFF(CURTIME(),'" + v2 + "') < 0)";
      */
            rule = "CALL spBetweenTimeCheck('" + v1 + "','" + v2 + "')";

        }

        action = "";

        query.clear();
        query.prepare("INSERT INTO ActionRules (name, ruleOrder, ruleType, andOrConnector, ruleActionText, rule, action, experimentalLocationId)\
                      VALUES (:name, :ruleOrder, :ruleType, :andOrConnector, :ruleActionText, :rule, :action, :locn)");
        query.bindValue(":name",sequenceName );
        query.bindValue(":ruleOrder",globalRuleCount++);
        query.bindValue(":ruleType","R");
        query.bindValue(":andOrConnector",0);     // default to no connector
        query.bindValue(":ruleActionText",ruletext);
        query.bindValue(":rule",rule);
        query.bindValue(":actionText",action);

        query.bindValue(":locn",experimentLocation);

        if (!query.exec())
        {
           qDebug() << "Can't add time to rules table!" << query.executedQuery();
        }
    }


    fillActionRuleTable(sequenceName);

    updateSequenceTable();


}


void MainWindow::on_ruleListView_clicked(QModelIndex index)
{
    index = index;
    if (ui->addRuleButton->isEnabled())
    {
       ui->delRuleButton->setEnabled(true);
       ui->addRuleButton->setEnabled(false);
       ui->addActionButton->setEnabled(false);
   }
   else
   {
       ui->delRuleButton->setEnabled(false);
       ui->addRuleButton->setEnabled(true);
       ui->addActionButton->setEnabled(true);
       ui->ruleListView->clearSelection();
   }



}

void MainWindow::on_delRuleButton_clicked()
{
    QSqlQuery query;

    QString sequenceName = ui->SeqComboBox->currentText();

    QStringList list;

    QModelIndex index =  ui->ruleListView->currentIndex();

    QVariant target = index.data().toString();

//    qDebug() << target.toString();

//    qDebug() <<  target.toString().section("::", 1, 1);

    QString delIitem = target.toString().section("::", 1, 1);

    query.prepare("DELETE FROM ActionRules WHERE name = :name AND ruleOrder = :del AND experimentalLocationId = :locn" );

    query.bindValue(":name",sequenceName);
    query.bindValue(":del",delIitem);

    query.bindValue(":locn",experimentLocation);


    if (!query.exec())
    {
        QMessageBox msgBox;
        msgBox.setIcon(QMessageBox::Critical);

        msgBox.setText("Database error - can't delete from ActionRule table!");
        msgBox.exec();

        qCritical("Cannot delete: %s (%s)",
                  db.lastError().text().toLatin1().data(),
                  qt_error_string().toLocal8Bit().data());
        return;

    }


    fillActionRuleTable(sequenceName);

    updateSequenceTable();





}



void MainWindow::on_InterruptcheckBox_clicked()
{
    updateSequenceTable();
}



void MainWindow::updateSequenceTable()
{

    QSqlQuery query;


    query.prepare("UPDATE Sequences SET priority = :priority, \
                  interruptable = :inter, ruleCount=:ruleCount, actionCount = :actionCount, schedulable=:sched,\
                  scenario = :scen, scenarioDescription = :scendesc WHERE name = :name AND experimentalLocationId = :locn");

    query.bindValue(":locn",experimentLocation);
    query.bindValue(":name",ui->SeqComboBox->currentText());
    query.bindValue(":priority",ui->prioritySpinBox->value());
    query.bindValue(":inter",ui->InterruptcheckBox->isChecked());
    query.bindValue(":ruleCount",globalRuleCount);
    query.bindValue(":actionCount",globalActionCount);
    query.bindValue(":sched",ui->scheduleCheckBox->isChecked());

//    qDebug()<<ui->seqTypeComboBox->currentText();
//    qDebug()<<ui->seqDescLineEdit->text();

    query.bindValue(":scen",ui->seqTypeComboBox->currentText());
    query.bindValue(":scendesc",ui->seqDescLineEdit->text());




    if (!query.exec())
    {

       qDebug() << query.lastQuery();

       QMessageBox msgBox;
       msgBox.setIcon(QMessageBox::Critical);

       msgBox.setText("Database error - can't add/update Sequence table!");
       msgBox.exec();

       qCritical("Cannot add/update: %s (%s)",
              db.lastError().text().toLatin1().data(),
              qt_error_string().toLocal8Bit().data());
       return;

     }

}

void MainWindow::on_prioritySpinBox_editingFinished()
{
        updateSequenceTable();
}

void MainWindow::on_livingSofa1CheckBox_toggled(bool checked)
{

    ui->LRS1OccupiedRadioButton->setEnabled(checked);
    ui->LRS1NotOccupiedRadioButton->setEnabled(checked);
    ui->LRS1ANDRadioButton->setEnabled(checked);
    ui->LRS1ORRadioButton->setEnabled(checked);

    if (checked)
    {
       ruleCount++;
    }
    else
    {
        ruleCount--;
    }
}

void MainWindow::on_livingSofa2CheckBox_toggled(bool checked)
{

    ui->LRS2OccupiedRadioButton->setEnabled(checked);
    ui->LRS2NotOccupiedRadioButton->setEnabled(checked);
    ui->LRS2ANDRadioButton->setEnabled(checked);
    ui->LRS2ORRadioButton->setEnabled(checked);

    if (checked)
    {
       ruleCount++;
    }
    else
    {
        ruleCount--;
    }
}

void MainWindow::on_livingSofa3CheckBox_toggled(bool checked)
{

    ui->LRS3OccupiedRadioButton->setEnabled(checked);
    ui->LRS3NotOccupiedRadioButton->setEnabled(checked);
    ui->LRS3ANDRadioButton->setEnabled(checked);
    ui->LRS3ORRadioButton->setEnabled(checked);

    if (checked)
    {
       ruleCount++;
    }
    else
    {
        ruleCount--;
    }
}

void MainWindow::on_livingSofa4CheckBox_toggled(bool checked)
{

    ui->LRS4OccupiedRadioButton->setEnabled(checked);
    ui->LRS4NotOccupiedRadioButton->setEnabled(checked);
    ui->LRS4ANDRadioButton->setEnabled(checked);
    ui->LRS4ORRadioButton->setEnabled(checked);

    if (checked)
    {
       ruleCount++;
    }
    else
    {
        ruleCount--;
    }
}

void MainWindow::on_livingSofa5CheckBox_toggled(bool checked)
{

    ui->LRS5OccupiedRadioButton->setEnabled(checked);
    ui->LRS5NotOccupiedRadioButton->setEnabled(checked);
    ui->LRS5ANDRadioButton->setEnabled(checked);
    ui->LRS5ORRadioButton->setEnabled(checked);

    if (checked)
    {
       ruleCount++;
    }
    else
    {
        ruleCount--;
    }
}

void MainWindow::on_TVcheckBox_toggled(bool checked)
{

    ui->TVSpinBox->setEnabled(checked);
    ui->TVANDRadioButton->setEnabled(checked);
    ui->TVORRadioButton->setEnabled(checked);

    if (checked)
    {
       ruleCount++;
    }
    else
    {
        ruleCount--;
    }

}



void MainWindow::on_moveRobotCheckBox_toggled(bool checked)
{
     ui->moveWaitGroupBox->setEnabled(checked);
     ui->moveRobotComboBox->setEnabled(checked);
     if (checked)
     {
         QString locQuery;
         locQuery = "SELECT  L1.locationId, L1.where, L2.where,\
                    L1.name, IF(STRCMP(L1.name,L2.name),L2.name,''),\
                    IF(STRCMP(L2.name,L3.name),L3.name,''),\
                    IF(STRCMP(L3.name,L4.name),L4.name,''), L1.orientation\
                   FROM Locations L1,\
                        Locations L2,\
                        Locations L3,\
                        Locations L4\
                   WHERE L2.locationId = L1.where\
                     AND L3.locationId = L2.where\
                     AND L4.locationId = L3.where\
                     AND L1.validRobotLocation = 1";

         if (experimentLocation == 1)
         {
             locQuery += " AND (L1.locationId < 500 OR L1.locationId = 999) ORDER BY L1.locationId";
         }

          if (experimentLocation == 2)
          {
              locQuery += " AND ((L1.locationId > 599 AND L1.locationId < 700) OR L1.locationId = 999) ORDER BY L1.locationId";
          }

          if (experimentLocation == 3)
          {
              locQuery += " AND ((L1.locationId > 699 AND L1.locationId < 800) OR L1.locationId = 999) ORDER BY L1.locationId";
          }
qDebug()<<locQuery;
         QSqlQuery query(locQuery);

         ui->moveRobotComboBox->clear();

         QString q1, q2, q3;

         q1 = q2 = q3 = "";

         int rows=0;
         int orientation = 0;

         while(query.next())
         {
             rows++;

             if (rows == 1)
             {
               orientation =  query.value(7).toInt();
             }

             q1 = q2 = q3 = "";

             if ( query.value(4).toString() != "")
             {
                 if ( query.value(1) == 0)
                 {
                     q1 = "";
                 }
                 else
                 {
                     q1 = " in the " +   query.value(4).toString();
                 }
             }

             if ( query.value(5).toString() != "")
             {
                 if ( query.value(2) == 0)
                 {
                     q2 = "";
                 }
                 else
                 {
                     q2 = " of the " +   query.value(5).toString();
                 }
             }

             ui->moveRobotComboBox->addItem( "::" + query.value(0).toString() + ":: " + query.value(3).toString() + q1 + q2 );
         }

        ui->moveRobotSpinBox->setValue(orientation);

        actionCount++;
     }
     else
     {
         actionCount--;
     }
}

void MainWindow::on_robotTrayCheckBox_toggled(bool checked)
{
         ui->robotTrayGroupBox->setEnabled(checked);

         if (checked)
         {
            actionCount++;
         }
         else
         {
             actionCount--;
         }
}

void MainWindow::on_addActionButton_clicked()
{
    // add new actions

    QString actiontext, rule, action;

    QString sequenceName = ui->SeqComboBox->currentText();
    QSqlQuery query;


    if (actionCount == 0)
    {
        QMessageBox msgBox;
        msgBox.setIcon(QMessageBox::Warning);

        msgBox.setText("No action has been specified!");
        msgBox.exec();
        return;
    }

    //----------------------------------------------------------------------
    if (ui->moveRobotCheckBox->isChecked())
    {

       actiontext = "move " + ui->robotComboBox->currentText() + " to " +  ui->moveRobotComboBox->currentText();


       QSqlQuery query("SELECT locationId, xCoord, yCoord,orientation FROM Locations where locationId = " + ui->moveRobotComboBox->currentText().section("::", 1, 1));

       while(query.next())
       {
           if ( query.value(0).toString() != ui->moveRobotComboBox->currentText().section("::", 1, 1))
           {
               QMessageBox msgBox;
               msgBox.setIcon(QMessageBox::Warning);

               msgBox.setText("Error on query to location table!");
               msgBox.exec();
               return;
           }
           else
           {
          //     double orient = ui->moveRobotSpinBox->value()/180.0 * 3.142;
               QString s;
               QString orientation;
          //     orientation.setNum(orient);
               orientation.setNum(ui->moveRobotSpinBox->value());  //save to DB in degrees
               s = "[" + query.value(1).toString() + ":" + query.value(2).toString() + ":" + orientation + "]";

               action = "base," + ui->robotComboBox->currentText().section("::", 1, 1) + "," + s + "," + ui->moveRobotComboBox->currentText().section("::", 1, 1);

           }


       }
       if (ui->moveWaitCheckBox->isChecked())
       {
            actiontext+=" and wait for completion";
            action +=   ",wait";
       }

       updateActionDB("base", sequenceName,actiontext,action);

    }
    //--------------------------------------------------------------------------------
    if (ui->robotTrayCheckBox->isChecked())
    {


       actiontext = "move tray on " + ui->robotComboBox->currentText() + " to ";

       if (ui->trayRaiseRadioButton->isChecked())
       {
            actiontext+="Raised";
            action = "tray," + ui->robotComboBox->currentText().section("::", 1, 1) + ",deliverup";
       }
       else
       {
             actiontext+="Lowered";
             action = "tray," + ui->robotComboBox->currentText().section("::", 1, 1) + ",store";
       }

       if (ui->trayWaitCheckBox->isChecked())
       {
            actiontext+=" and wait for completion";
            action +=   ",,wait";
       }

       updateActionDB("tray", sequenceName,actiontext,action);

    }


    //--------------------------------------------------------------------------------
    if (ui->armCheckBox->isChecked())
    {

       QString height;
       height.setNum(ui->armHeightSpinBox->value());

       actiontext = "arm to " + ui->armComboBox->currentText() + " height " + height ;
       action = "arm," + ui->robotComboBox->currentText().section("::", 1, 1) + ",trayToTable," + height;

       if (ui->armWaitCheckBox->isChecked())
       {
            actiontext+=" and wait for completion";
            action +=   ",wait";
       }

       updateActionDB("arm", sequenceName,actiontext,action);

    }

    //--------------------------------------------------------------------------------
    if (ui->apCheckBox->isChecked())
    {

       QString height;
       height.setNum(ui->apSpinBox->value());

       actiontext = "Change Action Possibility " + ui->apComboBox->currentText() + " to " + height ;

       action = "APoss,"+ ui->robotComboBox->currentText().section("::", 1, 1) + "," + ui->apComboBox->currentText().section("::", 1, 1) + "," + height;

       updateActionDB("APoss", sequenceName,actiontext,action);

    }

    //--------------------------------------------------------------------------
    if (ui->robotTorsoCheckBox->isChecked())
    {
        bool ruleSet = false;
        bool ruleSet2 = false;

        if (ui->torsoHomeCheckBox->isChecked())
        {
           actiontext = "move torso on " + ui->robotComboBox->currentText() + " to home position";
           action = "torso," + ui->robotComboBox->currentText().section("::", 1, 1) + ",home";
           ruleSet = true;
        }
        else
        {
            if (ui->torsoLookBackRadioButton->isChecked())
            {
               actiontext = "move torso on " + ui->robotComboBox->currentText() + " to the back position";
               action = "torso," + ui->robotComboBox->currentText().section("::", 1, 1) + ",back";
               ruleSet = true;
            }
            if (ui->torsoLookForwardRadioButton->isChecked())
            {
               actiontext = "move torso on " + ui->robotComboBox->currentText() + " to the forward position";
               action = "torso," + ui->robotComboBox->currentText().section("::", 1, 1) + ",front";
               ruleSet = true;
            }

            if (ui->torsoLookLeftRadioButton->isChecked())
            {
         //       if (ruleSet)
         //       {
         //           actiontext+= " and to the left";
         //           action += ",left";
         //           ruleSet2 = true;
         //       }
         //       else
         //       {
                    actiontext = "move torso on " + ui->robotComboBox->currentText() + " to shake";
                    action = "torso," + ui->robotComboBox->currentText().section("::", 1, 1) + ",shake";
                    ruleSet = true;
         //       }
            }

            if (ui->torsoLookRightRadioButton->isChecked())
            {
         //       if (ruleSet)
         //       {
         //           actiontext+= " and to the right";
         //           action += ",right";
         //           ruleSet2 = true;
         //      }
         //       else
         //       {
                    actiontext = "move torso on " + ui->robotComboBox->currentText() + " to the right";
                    action = "torso," + ui->robotComboBox->currentText().section("::", 1, 1) + ",right";
                    ruleSet = true;
          //      }
            }

        }

        if (!ruleSet)
        {
            QMessageBox msgBox;
            msgBox.setIcon(QMessageBox::Warning);

            msgBox.setText("No action has been specified for robot torso!");
            msgBox.exec();
            return;

        }

        if (ui->torsoWaitCheckBox->isChecked())
        {

             actiontext+=" and wait for completion";
             if (ruleSet2)
             {
                action +=   ",wait";
             }
             else
             {
                action +=   ",,wait";
             }

        }
        else
        {
             action +=   "";
        }

        updateActionDB("torso", sequenceName,actiontext,action);
    }

    //--------------------------------------------------------------------------------
    if (ui->robotEyesCheckBox->isChecked())
    {


       actiontext = "move eyes on " + ui->robotComboBox->currentText() + " to ";

       if (ui->eyesForwardRadioButton->isChecked())
       {
            actiontext+="front";
            action = "head," + ui->robotComboBox->currentText().section("::", 1, 1) + ",front";
       }
       else
       {
             actiontext+="back";
             action = "head," + ui->robotComboBox->currentText().section("::", 1, 1) + ",back";
       }

       if (ui->eyesWaitCheckBox->isChecked())
       {
            actiontext+=" and wait for completion";
            action +=   ",,wait";
       }

       updateActionDB("head", sequenceName,actiontext,action);

    }



    //-----------------------------------------------------

    if (ui->robotLightCheckBox->isChecked())
    {
        QString colour = ui->colourComboBox->currentText();
        actiontext = "Turn light on " + ui->robotComboBox->currentText() + " to  " + colour;
        action = "light," + ui->robotComboBox->currentText().section("::", 1, 1) + "," + colour;

        if (ui->colourWaitCheckBox->isChecked())
        {
             actiontext+=" and wait for completion";
             action +=   ",,wait";
        }

        updateActionDB("light", sequenceName,actiontext,action);

    }

    if (ui->robotExpressionCheckBox->isChecked())
    {
        int expression = ui->expressionComboBox->currentIndex();
        QString sExp;
        sExp.setNum(expression +1);
        actiontext = "Set expression on " + ui->robotComboBox->currentText() + " to " + ui->expressionComboBox->currentText();
        action = "expression," + ui->robotComboBox->currentText().section("::", 1, 1) + "," + sExp + "," + ui->expressionComboBox->currentText();

        updateActionDB("expression", sequenceName,actiontext,action);

    }

    //-------------------------------------------------------
    if (ui->robotSpeakCheckBox->isChecked())
    {
        QString said = ui->speakComboBox->currentText();

        if (said == "")
        {
            QMessageBox msgBox;
            msgBox.setIcon(QMessageBox::Warning);

            msgBox.setText("Nothing for robot to say!");
            msgBox.exec();
            return;

        }


        actiontext = ui->robotComboBox->currentText() + " says '" + said + "'";
        action = "speak," + ui->robotComboBox->currentText().section("::", 1, 1) + "," + said;

        if (ui->speakWaitCheckBox->isChecked())
        {
             actiontext+=" and wait for completion";
             action +=   ",,wait";
        }

        updateActionDB("speak", sequenceName,actiontext,action);
    }
    //-------------------------------------------------------
    if (ui->actionSequenceCheckBox->isChecked())
    {
        QString seq = ui->actionSequenceComboBox->currentText();

        if (seq == sequenceName )
        {
            QMessageBox msgBox;
            msgBox.setIcon(QMessageBox::Warning);

            msgBox.setText("This sequence and executed sequence cannot be the same (or we will be here all day!)");
            msgBox.exec();
            return;

        }

        actiontext = "Execute sequence '" + seq + "' on " + ui->robotComboBox->currentText();

        action = "sequence," + ui->robotComboBox->currentText().section("::", 1, 1) + "," + seq;

        updateActionDB("sequence", sequenceName,actiontext,action);
    }

    //----------------------------------------------------------
    if (ui->robotDelayCheckBox->isChecked())
    {

        QString v;

         v.setNum(ui->robotDelaySpinBox->value());

        actiontext = "Wait for " + v + " seconds on " + ui->robotComboBox->currentText();

        action = "sleep," + ui->robotComboBox->currentText().section("::", 1, 1) + "," + v;

        updateActionDB("sleep", sequenceName,actiontext,action);
    }
    //------------------------------------------


    if (ui->robotGUI->isChecked())
    {
        QSqlQuery query;
        int check = 0;
        int large = 0;

        if (ui->GUI1CheckBox->isChecked())
        {
            if (ui->GUI1ComboBox->currentText() == "")
            {
                QMessageBox msgBox;
                msgBox.setIcon(QMessageBox::Warning);

                msgBox.setText("You must provide a message for the robot to display!");
                msgBox.exec();
                return;
            }

            check+=1;

            if (ui->GUI1LargeCheckBox->isChecked()) large++;

        }

        if (ui->GUI2CheckBox->isChecked())
        {
            if (ui->GUI2ComboBox->currentText() == "")
            {
                QMessageBox msgBox;
                msgBox.setIcon(QMessageBox::Warning);

                msgBox.setText("You must provide a message for the robot to display!");
                msgBox.exec();
                return;
            }
            check+=2;
            if (ui->GUI2LargeCheckBox->isChecked()) large++;

        }

        if (ui->GUI3CheckBox->isChecked())
        {
            if (ui->GUI3ComboBox->currentText() == "")
            {
                QMessageBox msgBox;
                msgBox.setIcon(QMessageBox::Warning);

                msgBox.setText("You must provide a message for the robot to display!");
                msgBox.exec();
                return;
            }
            check+=4;
            if (ui->GUI3LargeCheckBox->isChecked()) large++;
        }

        if (ui->GUI4CheckBox->isChecked())
        {
            if (ui->GUI4ComboBox->currentText() == "")
            {
                QMessageBox msgBox;
                msgBox.setIcon(QMessageBox::Warning);

                msgBox.setText("You must provide a message for the robot to display!");
                msgBox.exec();
                return;
            }
            check+=8;
            if (ui->GUI4LargeCheckBox->isChecked()) large++;
        }

        // make sure that the messages are in sequence

        if (check == 1 || check == 3 || check == 7 || check == 15)
        {}
        else
        {
            QMessageBox msgBox;
            msgBox.setIcon(QMessageBox::Warning);

            msgBox.setText("Messages should have no gaps!");
            msgBox.exec();
            return;
        }

        // Only 1 expanded box can be checked

        if (large == 0 || large == 1)
        {}
        else
        {
            QMessageBox msgBox;
            msgBox.setIcon(QMessageBox::Warning);

            msgBox.setText("You can only expand 1 message!");
            msgBox.exec();
            return;
        }

        query.prepare("DELETE FROM userInterfaceGUI WHERE name = :name ");

        query.bindValue(":name",sequenceName);

        if (!query.exec())
        {
            qDebug() << query.lastQuery();

            QMessageBox msgBox;
            msgBox.setIcon(QMessageBox::Critical);

            msgBox.setText("Database error - can't delete fromm userInterfaceGUI table!");
            msgBox.exec();

            qCritical("Cannot delete: %s (%s)",
                   db.lastError().text().toLatin1().data(),
                   qt_error_string().toLocal8Bit().data());
            return;

        }

        query.clear();

        query.prepare("INSERT INTO userInterfaceGUI (name,guiMsg1,guiMsg1Enlarged,guiMsg1Execute,\
                                                          guiMsg2,guiMsg2Enlarged,guiMsg2Execute,\
                                                          guiMsg3,guiMsg3Enlarged,guiMsg3Execute,\
                                                          guiMsg4,guiMsg4Enlarged,guiMsg4Execute,guiMsgResult) \
                        VALUES(:name,:guiMsg1,:guiMsg1Enlarged,:guiMsg1Execute,\
                                     :guiMsg2,:guiMsg2Enlarged,:guiMsg2Execute,\
                                     :guiMsg3,:guiMsg3Enlarged,:guiMsg3Execute,\
                                     :guiMsg4,:guiMsg4Enlarged,:guiMsg4Execute,:guiMsgResult)");

        query.bindValue(":name",sequenceName);
        query.bindValue(":guiMsg1",ui->GUI1ComboBox->currentText());
        query.bindValue(":guiMsg1Enlarged",ui->GUI1LargeCheckBox->isChecked());
        query.bindValue(":guiMsg1Execute",ui->GUI1ExecuteComboBox->currentText());

        actiontext = ui->robotComboBox->currentText() + " GUI,"
                     + ui->GUI1ExecuteComboBox->currentText().trimmed();

        action = "GUI," + ui->robotComboBox->currentText().section("::", 1, 1) + ","
                        + ui->GUI1ExecuteComboBox->currentText().trimmed() + "@";



        if (ui->GUI2CheckBox->isChecked())
        {
          query.bindValue(":guiMsg2",ui->GUI2ComboBox->currentText());
          query.bindValue(":guiMsg2Enlarged",ui->GUI2LargeCheckBox->isChecked());
          query.bindValue(":guiMsg2Execute",ui->GUI2ExecuteComboBox->currentText());
          actiontext = ui->robotComboBox->currentText() + " GUI,"
                       + ui->GUI1ExecuteComboBox->currentText().trimmed() + ","
                       + ui->GUI2ExecuteComboBox->currentText().trimmed();
          action = "GUI," + ui->robotComboBox->currentText().section("::", 1, 1) + ","
                          + ui->GUI1ExecuteComboBox->currentText().trimmed() + "@"
                          + ui->GUI2ExecuteComboBox->currentText().trimmed() + "@";

        }
        else
        {
          query.bindValue(":guiMsg2","");
          query.bindValue(":guiMsg2Enlarged",0);
          query.bindValue(":guiMsg2Execute","");

        }

        if (ui->GUI3CheckBox->isChecked())
        {
         query.bindValue(":guiMsg3",ui->GUI3ComboBox->currentText());
         query.bindValue(":guiMsg3Enlarged",ui->GUI3LargeCheckBox->isChecked());
         query.bindValue(":guiMsg3Execute",ui->GUI3ExecuteComboBox->currentText());
         actiontext = ui->robotComboBox->currentText() + " GUI,"
                      + ui->GUI1ExecuteComboBox->currentText().trimmed() + ","
                      + ui->GUI2ExecuteComboBox->currentText().trimmed() + ","
                      + ui->GUI3ExecuteComboBox->currentText().trimmed();
         action = "GUI," + ui->robotComboBox->currentText().section("::", 1, 1) + ","
                         + ui->GUI1ExecuteComboBox->currentText().trimmed() + "@"
                         + ui->GUI2ExecuteComboBox->currentText().trimmed() + "@"
                         + ui->GUI3ExecuteComboBox->currentText().trimmed() + "@";
        }
        else
        {
          query.bindValue(":guiMsg3","");
          query.bindValue(":guiMsg3Enlarged",0);
          query.bindValue(":guiMsg3Execute","");
        }

        if (ui->GUI4CheckBox->isChecked())
        {
          query.bindValue(":guiMsg4",ui->GUI4ComboBox->currentText());
          query.bindValue(":guiMsg4Enlarged",ui->GUI4LargeCheckBox->isChecked());
          query.bindValue(":guiMsg4Execute",ui->GUI4ExecuteComboBox->currentText());
          actiontext = ui->robotComboBox->currentText() + " GUI,"
                       + ui->GUI1ExecuteComboBox->currentText().trimmed() + ","
                       + ui->GUI2ExecuteComboBox->currentText().trimmed() + ","
                       + ui->GUI3ExecuteComboBox->currentText().trimmed() + ","
                       + ui->GUI4ExecuteComboBox->currentText().trimmed();
          action = "GUI," + ui->robotComboBox->currentText().section("::", 1, 1) + ","
                          + ui->GUI1ExecuteComboBox->currentText().trimmed() + "@"
                          + ui->GUI2ExecuteComboBox->currentText().trimmed() + "@"
                          + ui->GUI3ExecuteComboBox->currentText().trimmed() + "@"
                          + ui->GUI4ExecuteComboBox->currentText().trimmed();
        }
        else
        {
          query.bindValue(":guiMsg4","");
          query.bindValue(":guiMsg4Enlarged",0);
          query.bindValue(":guiMsg4Execute","");
        }

        query.bindValue(":guiMsgResult",0);

        if (!query.exec())
        {
            qDebug() << query.lastQuery();

            QMessageBox msgBox;
            msgBox.setIcon(QMessageBox::Critical);

            msgBox.setText("Database error - can't insert into userInterfaceGUI table!");
            msgBox.exec();

            qCritical("Cannot insert: %s (%s)",
                       db.lastError().text().toLatin1().data(),
                       qt_error_string().toLocal8Bit().data());
            return;

        }

        updateActionDB("GUI", sequenceName, actiontext, action);
    }

    //-----------------------------------------------------

    if (ui->robotGoal1CheckBox->isChecked())
    {
        QString TF = ui->robotGoal1LineEdit->text();
        TF[0] = TF.at(0).toTitleCase();
        ui->robotGoal1LineEdit->setText(TF);


        actiontext = "SET " + ui->robotGoal1ComboBox->currentText() + " TO  " + TF;
        action = "cond," + ui->robotComboBox->currentText().section("::", 1, 1) + "," +
                                ui->robotGoal1ComboBox->currentText().section("::", 1, 1) + "," + TF;

        updateActionDB("cond", sequenceName,actiontext,action);

    }

   //-------------------------------------------------------

    fillActionRuleTable(sequenceName);

    updateSequenceTable();

}

void MainWindow::on_HotTapKitchenCheckBox_toggled(bool checked)
{

     ui->HTKTempSpinBox->setEnabled(checked);
     ui->HTKANDRadioButton->setEnabled(checked);
     ui->HTKORRadioButton->setEnabled(checked);

     if (checked)
     {
        ruleCount++;
     }
     else
     {
         ruleCount--;
     }

}

void MainWindow::on_ColdTapKitchenCheckBox_toggled(bool checked)
{
    ui->CTKTempSpinBox->setEnabled(checked);
    ui->CTKANDRadioButton->setEnabled(checked);
    ui->CTKORRadioButton->setEnabled(checked);

    if (checked)
    {
       ruleCount++;
    }
    else
    {
        ruleCount--;
    }
}

void MainWindow::on_KitchenCeilDoorLeftCheckBox_toggled(bool checked)
{
    ui->KCDLOpenRadioButton->setEnabled(checked);
    ui->KCDLClosedRadioButton->setEnabled(checked);
    ui->KCDLANDRadioButton->setEnabled(checked);
    ui->KCDLORRadioButton->setEnabled(checked);

    if (checked)
    {
       ruleCount++;
    }
    else
    {
        ruleCount--;
    }
}

void MainWindow::on_KitchenCeilDoorMiddleCheckBox_toggled(bool checked)
{
    ui->KCDMOpenRadioButton->setEnabled(checked);
    ui->KCDMClosedRadioButton->setEnabled(checked);
    ui->KCDMANDRadioButton->setEnabled(checked);
    ui->KCDMORRadioButton->setEnabled(checked);

    if (checked)
    {
       ruleCount++;
    }
    else
    {
        ruleCount--;
    }
}



void MainWindow::on_KitchenCeilDoorRightCheckBox_toggled(bool checked)
{
    ui->KCDROpenRadioButton->setEnabled(checked);
    ui->KCDRClosedRadioButton->setEnabled(checked);
    ui->KCDRANDRadioButton->setEnabled(checked);
    ui->KCDRORRadioButton->setEnabled(checked);

    if (checked)
    {
       ruleCount++;
    }
    else
    {
        ruleCount--;
    }
}

void MainWindow::on_KitchenFloorDrawerMiddleCheckBox_toggled(bool checked)
{
    ui->KFDMOpenRadioButton->setEnabled(checked);
    ui->KFDMClosedRadioButton->setEnabled(checked);
    ui->KFDMANDRadioButton->setEnabled(checked);
    ui->KFDMORRadioButton->setEnabled(checked);

    if (checked)
    {
       ruleCount++;
    }
    else
    {
        ruleCount--;
    }
}

void MainWindow::on_KitchenFloorDrawerRightCheckBox_toggled(bool checked)
{
    ui->KFDROpenRadioButton->setEnabled(checked);
    ui->KFDRClosedRadioButton->setEnabled(checked);
    ui->KFDRANDRadioButton->setEnabled(checked);
    ui->KFDRORRadioButton->setEnabled(checked);

    if (checked)
    {
       ruleCount++;
    }
    else
    {
        ruleCount--;
    }
}

void MainWindow::on_KitchenFloorDoorLeftCheckBox_toggled(bool checked)
{
    ui->KFDoorLOpenRadioButton->setEnabled(checked);
    ui->KFDoorLClosedRadioButton->setEnabled(checked);
    ui->KFDoorLANDRadioButton->setEnabled(checked);
    ui->KFDoorLORRadioButton->setEnabled(checked);

    if (checked)
    {
       ruleCount++;
    }
    else
    {
        ruleCount--;
    }
}

void MainWindow::on_KitchenFloorDoorMiddleCheckBox_toggled(bool checked)
{
    ui->KFDoorMOpenRadioButton->setEnabled(checked);
    ui->KFDoorMClosedRadioButton->setEnabled(checked);
    ui->KFDoorMANDRadioButton->setEnabled(checked);
    ui->KFDoorMORRadioButton->setEnabled(checked);

    if (checked)
    {
       ruleCount++;
    }
    else
    {
        ruleCount--;
    }
}

void MainWindow::on_KitchenFloorDoorRightCheckBox_toggled(bool checked)
{
    ui->KFDoorROpenRadioButton->setEnabled(checked);
    ui->KFDoorRClosedRadioButton->setEnabled(checked);
    ui->KFDoorRANDRadioButton->setEnabled(checked);
    ui->KFDoorRORRadioButton->setEnabled(checked);

    if (checked)
    {
       ruleCount++;
    }
    else
    {
        ruleCount--;
    }
}

void MainWindow::on_microwaveCheckBox_toggled(bool checked)
{
    ui->microwaveSpinBox->setEnabled(checked);
    ui->microwaveANDRadioButton->setEnabled(checked);
    ui->microwaveORRadioButton->setEnabled(checked);

    if (checked)
    {
       ruleCount++;
    }
    else
    {
        ruleCount--;
    }
}

void MainWindow::on_cookerCheckBox_toggled(bool checked)
{
    ui->cookerSpinBox->setEnabled(checked);
    ui->cookerANDRadioButton->setEnabled(checked);
    ui->cookerORRadioButton->setEnabled(checked);

    if (checked)
    {
       ruleCount++;
    }
    else
    {
        ruleCount--;
    }
}

void MainWindow::on_fridgeCheckBox_toggled(bool checked)
{
    ui->fridgeSpinBox->setEnabled(checked);
    ui->fridgeANDRadioButton->setEnabled(checked);
    ui->fridgeORRadioButton->setEnabled(checked);

    if (checked)
    {
       ruleCount++;
    }
    else
    {
        ruleCount--;
    }
}

void MainWindow::on_kettleCheckBox_toggled(bool checked)
{
    ui->KettleOffRadioButton->setEnabled(checked);
    ui->KettleOnRadioButton->setEnabled(checked);
    ui->kettleANDRadioButton->setEnabled(checked);
    ui->kettleORRadioButton->setEnabled(checked);

    if (checked)
    {
       ruleCount++;
    }
    else
    {
        ruleCount--;
    }
}

void MainWindow::on_toasterCheckBox_toggled(bool checked)
{
    ui->toasterSpinBox->setEnabled(checked);
    ui->toasterANDRadioButton->setEnabled(checked);
    ui->toasterORRadioButton->setEnabled(checked);

    if (checked)
    {
       ruleCount++;
    }
    else
    {
        ruleCount--;
    }
}

void MainWindow::on_dishwasherCheckBox_toggled(bool checked)
{
    ui->dishwasherSpinBox->setEnabled(checked);
    ui->dishwasherANDRadioButton->setEnabled(checked);
    ui->dishwasherORRadioButton->setEnabled(checked);

    if (checked)
    {
       ruleCount++;
    }
    else
    {
        ruleCount--;
    }
}

void MainWindow::on_doorbellCheckBox_toggled(bool checked)
{
    ui->doorbellSpinBox->setEnabled(checked);
    ui->doorbellANDRadioButton->setEnabled(checked);
    ui->doorbellORRadioButton->setEnabled(checked);

    if (checked)
    {
       ruleCount++;
    }
    else
    {
        ruleCount--;
    }
}

void MainWindow::on_timeCheckBox_toggled(bool checked)
{
    ui->timeEdit_1->setEnabled(checked);
    ui->timeAtRadioButton->setEnabled(checked);
    ui->timeBetweenRadioButton->setEnabled(checked);

    if (ui->timeAtRadioButton->isChecked())
    {
       ui->timeEdit_2->setEnabled(false);
    }
    else
    {
        ui->timeEdit_2->setEnabled(checked);
    }

    if (checked)
    {
       ruleCount++;
    }
    else
    {
        ruleCount--;
    }
}



void MainWindow::on_timeBetweenRadioButton_clicked()
{
           ui->timeEdit_2->setEnabled(true);
}

void MainWindow::on_timeAtRadioButton_clicked()
{
            ui->timeEdit_2->setEnabled(false);
}



void MainWindow::on_sensorActiveCheckBox_toggled(bool checked)
{
    ui->sensorActiveSpinBox->setEnabled(checked);


 //   if (checked)
 //   {
 //      ruleCount++;
 //   }
 //   else
 //   {
 //       ruleCount--;
 //   }
}

void MainWindow::on_lastActiveCheckBox_toggled(bool checked)
{
        ui->lastActiveSpinBox->setEnabled(checked);
}


void MainWindow::fillRuleActionTable(QString name, int Id, QString type, bool checkBox, int spinBox, bool ANDRadio , bool ORRadio)
{
    QString ruletext, rule, action;

    QString sequenceName = ui->SeqComboBox->currentText();
    QSqlQuery query;

    qDebug()<<name;
    qDebug()<<Id;
    qDebug()<<type;
    qDebug()<<checkBox;
    qDebug()<<spinBox;


    if (checkBox)
    {

        QString v;
        v.setNum(spinBox);


      if (ui->lastActiveCheckBox->isChecked())
      {
          QString v1;
          if (type == "Wattage" || type == "Temperature" || type == "Level")
          {
              ruletext = name + " Last " + type + " > " + v;
              rule = "SELECT * FROM Sensors WHERE sensorId = " + v1.setNum(Id) + " AND lastActiveValue > " + v;
          }
          else
          {

       /*       QString status;
              if (spinBox)
              {
                  status = type.section(":",0,0);
                  ruletext = name + " previously " + status;

                  if ( Id > 499 && Id < 600 )   // goals/conditions
                  {
                     rule = "SELECT * FROM Sensors WHERE sensorId = " + v1.setNum(Id) + " AND lastActiveValue = \"" + type + "\"";
                  }
                  else
                  {
                     rule = "SELECT * FROM Sensors WHERE sensorId = " + v1.setNum(Id) + " AND lastActiveValue = 1";
                 }
              }
              else
              {
                  status = type.section(":",1,1);
                  ruletext = name + " previously " + status;
                  rule = "SELECT * FROM Sensors WHERE sensorId = " + v1.setNum(Id) + " AND lastActiveValue = 0";
              }
*/
              QString status;
              if (spinBox)
              {
                  status = type.section(":",0,0);
                  ruletext = name + " previously " + status;
              }
              else
              {
                  status = type.section(":",1,1);
                  ruletext = name + " previously " + status;
              }

              if ( Id > 499 && Id < 600 )   // goals/conditions
              {
                 rule = "SELECT * FROM Sensors WHERE sensorId = "  + v1.setNum(Id) + " AND lastActiveValue = \"" + type + "\"";
              }
              else
              {
                  if (type == "On:Off")
                  {
                    rule = "SELECT * FROM Sensors WHERE sensorId = "  + v1.setNum(Id) + " AND lastStatus = \"" + status + "\"";
                  }
                  else
                  {
                    rule = "SELECT * FROM Sensors WHERE sensorId = "  + v1.setNum(Id) + " AND lastActiveValue = " + v;
                  }
              }


          }

          v1.setNum(ui->lastActiveSpinBox->value());
          ruletext += " AND was in this state within the last " + v1 + " seconds";
          rule+=" and lastUpdate+INTERVAL " + v1 + " SECOND >= NOW()";
      }
      else
      {
          QString v1;

          if (type == "Wattage" || type == "Temperature" || type == "Level")
          {
                   ruletext = name + " " + type + " > " + v;
                   rule = "SELECT * FROM Sensors WHERE sensorId = "  + v1.setNum(Id) + " AND value > " + v;
          }
          else
          {
              QString status;
              if (spinBox)
              {
                  status = type.section(":",0,0);
                  ruletext = name + " is " + status;
              }
              else
              {
                  status = type.section(":",1,1);
                  ruletext = name + " is " + status;
              }

              if ( Id > 499 && Id < 600 )   // goals/conditions
              {
                 rule = "SELECT * FROM Sensors WHERE sensorId = "  + v1.setNum(Id) + " AND value = \"" + type + "\"";
              }
              else
              {
                  if (type == "On:Off")
                  {
                    rule = "SELECT * FROM Sensors WHERE sensorId = "  + v1.setNum(Id) + " AND status = \"" + status + "\"";
                  }
                  else
                  {
                    rule = "SELECT * FROM Sensors WHERE sensorId = "  + v1.setNum(Id) + " AND value = " + v;
                  }
              }
          }


          if (ui->sensorActiveCheckBox->isChecked())
          {
             QString v1;
             v1.setNum(ui->sensorActiveSpinBox->value());
             ruletext += " AND has been in this state for more than " + v1 + " seconds";
             rule+=" and lastUpdate+INTERVAL " + v1 + " SECOND <= NOW()";
          }
      }

      action = "";

      query.clear();
      query.prepare("INSERT INTO ActionRules (name, ruleOrder, ruleType, andOrConnector, ruleActionText, rule, action, experimentalLocationId)\
                    VALUES (:name, :ruleOrder, :ruleType, :andOrConnector, :ruleActionText, :rule, :action, :locn)");
      query.bindValue(":name",sequenceName );
      query.bindValue(":ruleOrder",globalRuleCount++);
      query.bindValue(":ruleType","R");
      query.bindValue(":andOrConnector",0);     // default to no connector
      query.bindValue(":ruleActionText",ruletext);
      query.bindValue(":rule",rule);
      query.bindValue(":actionText",action);

      query.bindValue(":locn",experimentLocation);

      if (ruleCount > 1)
      {
         if (andCount+1 < ruleCount)
         {
            if (ANDRadio)
            {
               query.bindValue(":andOrConnector",1);
            }
            if (ORRadio)
            {
               query.bindValue(":andOrConnector",2);
            }
            andCount++;
         }
       }

      if (!query.exec())
      {
          qDebug() << "Can't add " + name + " to rules table!" << query.executedQuery();
      }
    }


}

void MainWindow::updateTime()
{
    ui->actualTime->clear();
    ui->schedulerTime->clear();

    QSqlQuery query;

    query.clear();

    query.prepare("SELECT time(NOW())");

    query.exec();

    while(query.next())
    {
        ui->actualTime->setText(query.value(0).toString());
    }

    query.clear();

 //   query.prepare("CALL getSchedulerTime()");

    query.exec("CALL spGetSchedulerTime()");

    while(query.next())
    {
      ui->schedulerTime->setText(query.value(0).toString());
    }

}

void MainWindow::resetGui()
{
    ruleCount=0;
    actionCount=0;
    QSqlQuery query;



    ui->userLocationANDradioButton->setEnabled(false);
    ui->userLocationANDradioButton->setChecked(true);
    ui->userLocationORradioButton->setEnabled(false);
    ui->userLocationComboBox->clear();
    ui->userLocationUserComboBox->setEnabled(true);
    ui->userLocationUserComboBox->clear();
    
    query.clear();

    query.prepare("SELECT userId, nickname FROM Users where userId = :userId");

    query.bindValue(":userId",defaultUserId);

    query.exec();

    ui->robotLocationRobotComboBox->clear();

    while(query.next())
    {
        ui->userLocationUserComboBox->addItem("::"+ query.value(0).toString() + "::" + query.value(1).toString());
    }

    ui->robotLocationANDradioButton->setEnabled(false);
    ui->robotLocationANDradioButton->setChecked(true);
    ui->robotLocationORradioButton->setEnabled(false);
    ui->robotLocationRobotComboBox->setEnabled(false);
    ui->robotLocationComboBox->clear();
    ui->robotLocationRobotComboBox->clear();






    ui->sensorTab->setEnabled(false);

    ui->DRS1OccupiedRadioButton->setEnabled(false);
    ui->DRS1NotOccupiedRadioButton->setEnabled(false);
    ui->DRS1NotOccupiedRadioButton->setChecked(true);

    ui->DRS1ANDradioButton->setEnabled(false);
    ui->DRS1ANDradioButton->setChecked(true);
    ui->DRS1ORradioButton->setEnabled(false);

    ui->DRS2OccupiedRadioButton->setEnabled(false);
    ui->DRS2NotOccupiedRadioButton->setEnabled(false);
    ui->DRS2NotOccupiedRadioButton->setChecked(true);

    ui->DRS2ANDradioButton->setEnabled(false);
    ui->DRS2ANDradioButton->setChecked(true);
    ui->DRS2ORradioButton->setEnabled(false);

    ui->BCDBOpenRadioButton->setEnabled(false);
    ui->BCDBClosedRadioButton->setEnabled(false);
    ui->BCDBClosedRadioButton->setChecked(true);

    ui->BCDBANDRadioButton->setEnabled(false);
    ui->BCDBANDRadioButton->setChecked(true);
    ui->BCDBORRadioButton->setEnabled(false);

    ui->BCDTOpenRadioButton->setEnabled(false);
    ui->BCDTClosedRadioButton->setEnabled(false);
    ui->BCDTClosedRadioButton->setChecked(true);

    ui->BCDTANDRadioButton->setEnabled(false);
    ui->BCDTANDRadioButton->setChecked(true);
    ui->BCDTORRadioButton->setEnabled(false);

    ui->SCDLOpenRadioButton->setEnabled(false);
    ui->SCDLClosedRadioButton->setEnabled(false);
    ui->SCDLClosedRadioButton->setChecked(true);

    ui->SCDLANDRadioButton->setEnabled(false);
    ui->SCDLANDRadioButton->setChecked(true);
    ui->SCDLORRadioButton->setEnabled(false);

    ui->SCDROpenRadioButton->setEnabled(false);
    ui->SCDRClosedRadioButton->setEnabled(false);
    ui->SCDRClosedRadioButton->setChecked(true);

    ui->SCDRANDRadioButton->setEnabled(false);
    ui->SCDRANDRadioButton->setChecked(true);
    ui->SCDRORRadioButton->setEnabled(false);

    ui->SCDBOpenRadioButton->setEnabled(false);
    ui->SCDBClosedRadioButton->setEnabled(false);
    ui->SCDBClosedRadioButton->setChecked(true);

    ui->SCDBANDRadioButton->setEnabled(false);
    ui->SCDBANDRadioButton->setChecked(true);
    ui->SCDBORRadioButton->setEnabled(false);

    ui->SCDMOpenRadioButton->setEnabled(false);
    ui->SCDMClosedRadioButton->setEnabled(false);
    ui->SCDMClosedRadioButton->setChecked(true);

    ui->SCDMANDRadioButton->setEnabled(false);
    ui->SCDMANDRadioButton->setChecked(true);
    ui->SCDMORRadioButton->setEnabled(false);

    ui->SCDTOpenRadioButton->setEnabled(false);
    ui->SCDTClosedRadioButton->setEnabled(false);
    ui->SCDTClosedRadioButton->setChecked(true);

    ui->SCDTANDRadioButton->setEnabled(false);
    ui->SCDTANDRadioButton->setChecked(true);
    ui->SCDTORRadioButton->setEnabled(false);


    ui->LRS1OccupiedRadioButton->setEnabled(false);
    ui->LRS1NotOccupiedRadioButton->setEnabled(false);
    ui->LRS1NotOccupiedRadioButton->setChecked(true);

    ui->LRS1ANDRadioButton->setEnabled(false);
    ui->LRS1ANDRadioButton->setChecked(true);
    ui->LRS1ORRadioButton->setEnabled(false);

    ui->LRS2OccupiedRadioButton->setEnabled(false);
    ui->LRS2NotOccupiedRadioButton->setEnabled(false);
    ui->LRS2NotOccupiedRadioButton->setChecked(true);

    ui->LRS2ANDRadioButton->setEnabled(false);
    ui->LRS2ANDRadioButton->setChecked(true);
    ui->LRS2ORRadioButton->setEnabled(false);

    ui->LRS3OccupiedRadioButton->setEnabled(false);
    ui->LRS3NotOccupiedRadioButton->setEnabled(false);
    ui->LRS3NotOccupiedRadioButton->setChecked(true);

    ui->LRS3ANDRadioButton->setEnabled(false);
    ui->LRS3ANDRadioButton->setChecked(true);
    ui->LRS3ORRadioButton->setEnabled(false);

    ui->LRS4OccupiedRadioButton->setEnabled(false);
    ui->LRS4NotOccupiedRadioButton->setEnabled(false);
    ui->LRS4NotOccupiedRadioButton->setChecked(true);

    ui->LRS4ANDRadioButton->setEnabled(false);
    ui->LRS4ANDRadioButton->setChecked(true);
    ui->LRS4ORRadioButton->setEnabled(false);

    ui->LRS5OccupiedRadioButton->setEnabled(false);
    ui->LRS5NotOccupiedRadioButton->setEnabled(false);
    ui->LRS5NotOccupiedRadioButton->setChecked(true);

    ui->LRS5ANDRadioButton->setEnabled(false);
    ui->LRS5ANDRadioButton->setChecked(true);
    ui->LRS5ORRadioButton->setEnabled(false);


    ui->KCDLOpenRadioButton->setEnabled(false);
    ui->KCDLClosedRadioButton->setEnabled(false);
    ui->KCDLClosedRadioButton->setChecked(true);

    ui->KCDLANDRadioButton->setEnabled(false);
    ui->KCDLANDRadioButton->setChecked(true);
    ui->KCDLORRadioButton->setEnabled(false);

    ui->KCDMOpenRadioButton->setEnabled(false);
    ui->KCDMClosedRadioButton->setEnabled(false);
    ui->KCDMClosedRadioButton->setChecked(true);

    ui->KCDMANDRadioButton->setEnabled(false);
    ui->KCDMANDRadioButton->setChecked(true);
    ui->KCDMORRadioButton->setEnabled(false);

    ui->KCDROpenRadioButton->setEnabled(false);
    ui->KCDRClosedRadioButton->setEnabled(false);
    ui->KCDRClosedRadioButton->setChecked(true);

    ui->KCDRANDRadioButton->setEnabled(false);
    ui->KCDRANDRadioButton->setChecked(true);
    ui->KCDRORRadioButton->setEnabled(false);

    ui->KFDMOpenRadioButton->setEnabled(false);
    ui->KFDMClosedRadioButton->setEnabled(false);
    ui->KFDMClosedRadioButton->setChecked(true);

    ui->KFDMANDRadioButton->setEnabled(false);
    ui->KFDMANDRadioButton->setChecked(true);
    ui->KFDMORRadioButton->setEnabled(false);

    ui->KFDROpenRadioButton->setEnabled(false);
    ui->KFDRClosedRadioButton->setEnabled(false);
    ui->KFDRClosedRadioButton->setChecked(true);

    ui->KFDRANDRadioButton->setEnabled(false);
    ui->KFDRANDRadioButton->setChecked(true);
    ui->KFDRORRadioButton->setEnabled(false);


    ui->KFDoorLOpenRadioButton->setEnabled(false);
    ui->KFDoorLClosedRadioButton->setEnabled(false);
    ui->KFDoorLClosedRadioButton->setChecked(true);

    ui->KFDoorLANDRadioButton->setEnabled(false);
    ui->KFDoorLANDRadioButton->setChecked(true);
    ui->KFDoorLORRadioButton->setEnabled(false);


    ui->KFDoorMOpenRadioButton->setEnabled(false);
    ui->KFDoorMClosedRadioButton->setEnabled(false);
    ui->KFDoorMClosedRadioButton->setChecked(true);

    ui->KFDoorMANDRadioButton->setEnabled(false);
    ui->KFDoorMANDRadioButton->setChecked(true);
    ui->KFDoorMORRadioButton->setEnabled(false);


    ui->KFDoorROpenRadioButton->setEnabled(false);
    ui->KFDoorRClosedRadioButton->setEnabled(false);
    ui->KFDoorRClosedRadioButton->setChecked(true);

    ui->KFDoorRANDRadioButton->setEnabled(false);
    ui->KFDoorRANDRadioButton->setChecked(true);
    ui->KFDoorRORRadioButton->setEnabled(false);

    ui->TVSpinBox->setEnabled(false);
    ui->TVANDRadioButton->setEnabled(false);
    ui->TVANDRadioButton->setChecked(true);
    ui->TVORRadioButton->setEnabled(false);

    ui->microwaveSpinBox->setEnabled(false);
    ui->microwaveANDRadioButton->setEnabled(false);
    ui->microwaveANDRadioButton->setChecked(true);
    ui->microwaveORRadioButton->setEnabled(false);

    ui->cookerSpinBox->setEnabled(false);
    ui->cookerANDRadioButton->setEnabled(false);
    ui->cookerANDRadioButton->setChecked(true);
    ui->cookerORRadioButton->setEnabled(false);

    ui->toasterSpinBox->setEnabled(false);
    ui->toasterANDRadioButton->setEnabled(false);
    ui->toasterANDRadioButton->setChecked(true);
    ui->toasterORRadioButton->setEnabled(false);

    ui->fridgeSpinBox->setEnabled(false);
    ui->fridgeANDRadioButton->setEnabled(false);
    ui->fridgeANDRadioButton->setChecked(true);
    ui->fridgeORRadioButton->setEnabled(false);

    ui->dishwasherSpinBox->setEnabled(false);
    ui->dishwasherANDRadioButton->setEnabled(false);
    ui->dishwasherANDRadioButton->setChecked(true);
    ui->dishwasherORRadioButton->setEnabled(false);

    ui->KettleOffRadioButton->setEnabled(false);
    ui->KettleOnRadioButton->setEnabled(false);
    ui->KettleOffRadioButton->setChecked(true);

    ui->kettleANDRadioButton->setEnabled(false);
    ui->kettleANDRadioButton->setChecked(true);
    ui->kettleORRadioButton->setEnabled(false);

    ui->doorbellSpinBox->setEnabled(false);
    ui->doorbellANDRadioButton->setEnabled(false);
    ui->doorbellANDRadioButton->setChecked(true);
    ui->doorbellORRadioButton->setEnabled(false);


    ui->HTKTempSpinBox->setEnabled(false);
    ui->HTKANDRadioButton->setEnabled(false);
    ui->HTKANDRadioButton->setChecked(true);
    ui->HTKORRadioButton->setEnabled(false);


    ui->CTKTempSpinBox->setEnabled(false);
    ui->CTKANDRadioButton->setEnabled(false);
    ui->CTKANDRadioButton->setChecked(true);
    ui->CTKORRadioButton->setEnabled(false);


    ui->Goal1TRUERadioButton->setEnabled(false);
    ui->Goal1FALSERadioButton->setEnabled(false);
    ui->Goal1FALSERadioButton->setChecked(true);
    ui->Goal1ANDRadioButton->setEnabled(false);
    ui->Goal1ANDRadioButton->setChecked(true);
    ui->Goal1ORRadioButton->setEnabled(false);
    ui->Goal1ComboBox->setEnabled(false);
    ui->Goal1LineEdit->setEnabled(false);



    ui->timeEdit_1->setEnabled(false);
    ui->timeAtRadioButton->setEnabled(false);
    ui->timeAtRadioButton->setChecked(true);
    ui->timeBetweenRadioButton->setEnabled(false);
    ui->timeEdit_2->setEnabled(false);

    ui->sensorActiveSpinBox->setEnabled(false);
    ui->lastActiveSpinBox->setEnabled(false);

    // ZUYD Sensors

    ui->HUYTLRS1OccupiedRadioButton->setEnabled(false);
    ui->HUYTLRS1NotOccupiedRadioButton->setEnabled(false);
    ui->HUYTLRS1NotOccupiedRadioButton->setChecked(true);

    ui->HUYTLRS1ANDRadioButton->setEnabled(false);
    ui->HUYTLRS1ANDRadioButton->setChecked(true);
    ui->HUYTLRS1ORRadioButton->setEnabled(false);

    ui->HUYTLRS2OccupiedRadioButton->setEnabled(false);
    ui->HUYTLRS2NotOccupiedRadioButton->setEnabled(false);
    ui->HUYTLRS2NotOccupiedRadioButton->setChecked(true);

    ui->HUYTLRS2ANDRadioButton->setEnabled(false);
    ui->HUYTLRS2ANDRadioButton->setChecked(true);
    ui->HUYTLRS2ORRadioButton->setEnabled(false);

    ui->HUYTLRS3OccupiedRadioButton->setEnabled(false);
    ui->HUYTLRS3NotOccupiedRadioButton->setEnabled(false);
    ui->HUYTLRS3NotOccupiedRadioButton->setChecked(true);

    ui->HUYTLRS3ANDRadioButton->setEnabled(false);
    ui->HUYTLRS3ANDRadioButton->setChecked(true);
    ui->HUYTLRS3ORRadioButton->setEnabled(false);

    ui->HUYTCupEmptyRadioButton->setEnabled(false);
    ui->HUYTCupFullRadioButton->setEnabled(false);
    ui->cupLevelANDRadioButton->setEnabled(false);
    ui->cupLevelANDRadioButton->setChecked(true);
    ui->cupLevelORRadioButton->setEnabled(false);
    ui->HUYTCupEmptyRadioButton->setChecked(true);

    ui->ZUYDDoorbellOffRadioButton->setEnabled(false);
    ui->ZUYDDoorbellOnRadioButton->setEnabled(false);
    ui->ZUYDDoorbellOffRadioButton->setChecked(true);

    ui->ZUYDDoorbellANDRadioButton->setEnabled(false);
    ui->ZUYDDoorbellANDRadioButton->setChecked(true);
    ui->ZUYDDoorbellORRadioButton->setEnabled(false);

    ui->ZUYDFridgeOffRadioButton->setEnabled(false);
    ui->ZUYDFridgeOnRadioButton->setEnabled(false);
    ui->ZUYDFridgeOffRadioButton->setChecked(true);

    ui->ZUYDFridgeANDRadioButton->setEnabled(false);
    ui->ZUYDFridgeANDRadioButton->setChecked(true);
    ui->ZUYDFridgeORRadioButton->setEnabled(false);

    // end ZUYD Sensors



    query.clear();

    query.prepare("SELECT * FROM Sequences WHERE experimentalLocationId = :locn order by name");
    query.bindValue(":locn",experimentLocation);

    query.exec();

     ui->SeqComboBox->clear();

     while(query.next())
     {
         ui->SeqComboBox->addItem(query.value(0).toString());
     }

     ui->SeqComboBox->clearEditText();

     ui->seqTypeComboBox->clear();
     ui->seqTypeComboBox->addItem("High Level");
     ui->seqTypeComboBox->addItem("user");
     ui->seqTypeComboBox->addItem("Mid level");
     ui->seqTypeComboBox->addItem("Low Level");
     ui->seqTypeComboBox->addItem("Protected");
     ui->seqTypeComboBox->addItem("scenario1");
     ui->seqTypeComboBox->addItem("scenario2");
     ui->seqTypeComboBox->addItem("scenario3");
     ui->seqTypeComboBox->addItem("scenario4");
     ui->seqTypeComboBox->addItem("scenario5");
     ui->seqTypeComboBox->addItem("scenario6");


     ui->prioritySpinBox->setEnabled(false);
     ui->InterruptcheckBox->setEnabled(false);
     ui->scheduleCheckBox->setEnabled(false);
     ui->seqAddButton->setEnabled(false);
     ui->seqDelButton->setEnabled(false);

     ui->seqDescLineEdit->setEnabled(false);
     ui->seqTypeComboBox->setEnabled(false);

     globalRuleCount = 0;
     globalActionCount = 0;



     ui->userLocationGroupBox->setEnabled(false);
     ui->robotLocationGroupBox->setEnabled(false);

     ui->sensorGroupBox->setEnabled(false);
     ui->timeGroupBox->setEnabled(false);
     ui->sensorActiveGroupBox->setEnabled(false);
     ui->lastActiveGroupBox->setEnabled(false);


     ui->actionGroupBox->setEnabled(false);




     query.clear();

     query.prepare("SELECT robotId, robotName FROM Robot");

     query.exec();

     ui->robotComboBox->clear();

     while(query.next())
     {
         ui->robotComboBox->addItem("::"+ query.value(0).toString() + "::" + query.value(1).toString());
     }

     ui->robotComboBox->setCurrentIndex(3);
     ui->trayRaiseRadioButton->setChecked(false);
     ui->trayLowerRadioButton->setChecked(true);
     ui->robotTrayGroupBox->setEnabled(false);

     ui->TorsoGroupBox->setEnabled(false);
     ui->moveWaitGroupBox->setEnabled(false);
     ui->torsoHomeCheckBox->setChecked(true);


  //   ui->robotEyesCheckBox->setEnabled(false);
     ui->robotEyesGroupBox->setEnabled(false);



     ui->eyesForwardRadioButton->setChecked(true);
     ui->eyesBackRadioButton->setChecked(false);

     ui->robotArmGroupBox->setEnabled(false);
     ui->armComboBox->addItem("Grasp from tray, place on table");
     ui->armHeightSpinBox->setValue(0.45);
     ui->armWaitCheckBox->setChecked(true);

     ui->apGroupBox->setEnabled(false);

     ui->apComboBox->clear();




     ui->speakGroupBox->setEnabled(false);

     ui->colourComboBox->clear();
     ui->colourComboBox->addItem("red");
     ui->colourComboBox->addItem("green");
     ui->colourComboBox->addItem("yellow");
     ui->colourComboBox->addItem("blue");
     ui->colourComboBox->addItem("white");

      ui->colourGroupBox->setEnabled(false);


      ui->expressionComboBox->clear();
      ui->expressionComboBox->addItem("basic");
      ui->expressionComboBox->addItem("sad");
      ui->expressionComboBox->addItem("fear");
      ui->expressionComboBox->addItem("disgust");
      ui->expressionComboBox->addItem("angry");
      ui->expressionComboBox->addItem("joy");
      ui->expressionComboBox->addItem("surprised");
      ui->expressionComboBox->addItem("low batteries");
      ui->expressionComboBox->addItem("squeeze");

      ui->expressionGroupBox->setEnabled(false);

     ui->actionSequenceComboBox->setEnabled(false);

     ui->robotDelaySpinBox->setEnabled(false);

     ui->robotGUI->setChecked(false);
     ui->robotGUIgroupBox->setEnabled(false);

     ui->robotGoal1LineEdit->setEnabled(false);
     ui->robotGoal1ComboBox->setEnabled(false);


     fillActionRuleTable("");

  //   ui->pythonCreatePushButton->setEnabled(false);
     ui->addActionButton->setEnabled(false);
     ui->addRuleButton->setEnabled(false);
     ui->delRuleButton->setEnabled(false);

     ui->QBpushButton->setEnabled(false);
     ui->pythonCreatePushButton->setEnabled(false);

}


void MainWindow::on_robotTorsoCheckBox_toggled(bool checked)
{
     ui->TorsoGroupBox->setEnabled(checked);

     if (ui->torsoHomeCheckBox->isChecked())
     {

          ui->TorsoBFGroupBox->setEnabled(false);
          ui->TorsoLRGroupBox->setEnabled(false);
     }

     if (checked)
     {
         actionCount++;
     }
     else
     {
         actionCount--;
     }
}

void MainWindow::on_torsoHomeCheckBox_toggled(bool checked)
{
    ui->TorsoLRGroupBox->setEnabled(!checked);
    ui->TorsoBFGroupBox->setEnabled(!checked);
}

void MainWindow::on_torsoLookBackRadioButton_toggled(bool checked)
{
    if (checked)
    {
        ui->torsoLookForwardRadioButton->setChecked(false);
    }
}


void MainWindow::on_torsoLookForwardRadioButton_toggled(bool checked)
{
    if (checked)
    {
        ui->torsoLookBackRadioButton->setChecked(false);
    }
}

void MainWindow::on_torsoLookRightRadioButton_toggled(bool checked)
{
    if (checked)
    {
        ui->torsoLookLeftRadioButton->setChecked(false);
    }
}

void MainWindow::on_torsoLookLeftRadioButton_toggled(bool checked)
{
    if (checked)
    {
        ui->torsoLookRightRadioButton->setChecked(false);
    }
}


void MainWindow::updateActionDB(QString effector, QString sequenceName, QString actiontext, QString action)
{
    qDebug()<<sequenceName;
    qDebug()<<globalRuleCount;
    qDebug()<<actiontext;
    qDebug()<<action;

    QSqlQuery query;
    QString rule = "";

    query.clear();
    query.prepare("INSERT INTO ActionRules (name, ruleOrder, ruleType, andOrConnector, ruleActionText, rule, action, experimentalLocationId)\
                  VALUES (:name, :ruleOrder, :ruleType, :andOrConnector, :ruleActionText, :rule, :action, :locn)");

    query.bindValue(":name",sequenceName );
    query.bindValue(":ruleOrder",globalRuleCount++);
    query.bindValue(":ruleType","A");
    query.bindValue(":andOrConnector",0);     // default to no connector
    query.bindValue(":ruleActionText",actiontext);
    query.bindValue(":rule",rule);
    query.bindValue(":actionText",action);
    query.bindValue(":locn",experimentLocation);

    if (!query.exec())
    {
        qDebug() << "Can't add " + effector + " to actionrules table!" << query.executedQuery();
    }
}

void MainWindow::on_robotLightCheckBox_toggled(bool checked)
{
    ui->colourGroupBox->setEnabled(checked);

    if (checked)
    {
        actionCount++;
    }
    else
    {
        actionCount--;
    }
}

void MainWindow::on_robotSpeakCheckBox_toggled(bool checked)
{
    ui->speakGroupBox->setEnabled(checked);


    QString userStr = ui->userLocationUserComboBox->currentText();

    int userId;

    if (userStr =="")
    {
        userId=1; //defaults to first user who is english
    }
    else
    {
        userId = userStr.section("::",1,1).toInt();
    }
 //   qDebug()<<userStr;
 //   qDebug()<<userId;

    QString userIdStr;
    userIdStr.setNum(userId);

    QString Qry = "SELECT M.message FROM Messages M, Users U where M.languageId = U.languageId and U.userId = " + userIdStr;

 //   qDebug()<<Qry;

    if (checked)
    {
        QSqlQuery query(Qry);

        ui->speakComboBox->clear();

        while(query.next())
        {
            ui->speakComboBox->addItem(query.value(0).toString());
        }

        ui->speakComboBox->clearEditText();


        actionCount++;
    }
    else
    {
        actionCount--;
    }
}

void MainWindow::on_actionSequenceCheckBox_toggled(bool checked)
{
    ui->actionSequenceComboBox->setEnabled(checked);

    if (checked)
    {
        QString seqQuery;
        QString locn;
        locn.setNum(experimentLocation);
        seqQuery = "SELECT * FROM Sequences WHERE experimentalLocationId = " + locn +
                   " order by name";

        QSqlQuery query(seqQuery);


        ui->actionSequenceComboBox->clear();

        while(query.next())
        {   qDebug()<< query.value(0).toString();
            ui->actionSequenceComboBox->addItem(query.value(0).toString());
        }

        ui->actionSequenceComboBox->clearEditText();


        actionCount++;
    }
    else
    {
        actionCount--;
    }
}

void MainWindow::on_robotDelayCheckBox_toggled(bool checked)
{
    ui->robotDelaySpinBox->setEnabled(checked);

    if (checked)
    {
       actionCount++;
    }
    else
    {
        actionCount--;
    }
}

void MainWindow::on_pythonCreatePushButton_clicked()
{
  //  QString fileName = QFileDialog::getOpenFileName(this,
  //       tr("Open Python Script"), "/home/joe/git/care-o-bot/cob-script-server-tutorial/scripts/",
  //                  tr("Python Scripts (*.py)"));


    if (globalRuleCount == 0 && globalActionCount == 0)
    {
        QMessageBox msgBox;
        msgBox.setIcon(QMessageBox::Warning);

        msgBox.setText("No rules or actions have been specified!");
        msgBox.exec();
        return;
    }

    QString dir = "/home/joe/QTProjects/COBSequencer/scripts/";
    QString sequenceName = ui->SeqComboBox->currentText();

    QString saveFile = dir + sequenceName + ".py";

  //  qDebug()<<"DIR-> "<<saveFile;

    QString fileName = QFileDialog::getSaveFileName(this,
         tr("Open Python Script"), saveFile,
                    tr("Python Scripts (*.py)"));

 //   qDebug()<<"Selected-> " << fileName;

    QSqlQuery query;

    // count the number of rules and actions

    //int localActionCount = 0;
    int localRuleCount = 0;

    query.clear();


    query.prepare("SELECT * FROM ActionRules where name=:name and ruleType = :ruleStr and ExperimentalLocationId = :locn");

    query.bindValue(":locn",experimentLocation);
    query.bindValue(":name",sequenceName );
    query.bindValue(":ruleStr","R");

    if (!query.exec())
    {
        qDebug() << "Can't select from actionrules table!" << query.executedQuery();
        return;
    }

    localRuleCount = query.size();
 //   qDebug()<<  localRuleCount ;

    query.clear();

    query.prepare("SELECT * FROM  ActionRules WHERE name = :name and ExperimentalLocationId = :locn ORDER BY ruleOrder");

    query.bindValue(":locn",experimentLocation);
    query.bindValue(":name",sequenceName );

    if (!query.exec())
    {
        qDebug() << "Can't select from actionrules table!" << query.executedQuery();
        return;
    }


    QFile file(fileName);
    if (!file.open(QIODevice::WriteOnly))
    {
        qDebug()<< "Cannot open file for writing: " << qPrintable(file.errorString());
        return;
    }

    QTextStream out(&file);


    int imageHistory = QMessageBox::question(this, tr("COB Sequencer"),
                                    tr("Do you want to include image history?"),
                                    QMessageBox::Yes | QMessageBox::No, QMessageBox::No);

    int initialisation = QMessageBox::question(this, tr("COB Sequencer"),
                                    tr("Do you want to include component initialisation?"),
                                    QMessageBox::Yes | QMessageBox::No, QMessageBox::No);

    // imports

    out << "#!/usr/bin/python" << endl;
    out << "" << endl;
    out << "### This is an auto-generated script - do not edit ###" << endl;
    out << "" << endl;
    out << "import subprocess" << endl;
    out << "import time" << endl;
    out << "import sys" << endl;
    out << "sys.path.append('/home/joe/QTProjects/Core/')" << endl;

    out << "import roslib" << endl;
    out << "roslib.load_manifest('cob_script_server')" << endl;
    out << "import rospy" << endl;
    out << "" << endl;
    out << "from simple_script_server import script" << endl;
    out << "from config import *            # contains configuration details for UH Robot House inc. DB login info" << endl;
    out << "" << endl;
    out << "import MySQLdb" << endl;

    if (imageHistory == QMessageBox::Yes)
    {
       out << "import history" << endl;
    }

    out << "" << endl;

    // main class - initialise and start sql

    out << "#--------------------------------------------------------------------------------" << endl;
    out << "class " << sequenceName << "(script):" << endl;
    out << "" << endl;

    out << "  def Initialize(self):" << endl;
    out << "" << endl;
    out << "    if not self.sss.parse:" << endl;
    out << "       rospy.loginfo(\" *********** This is the " << sequenceName << " script ************** \")" << endl;
    out << "" << endl;

    if (initialisation == QMessageBox::Yes)
    {
       out << "    if not self.sss.parse:" << endl;
       out << "       rospy.loginfo(\"Initializing all components...\")" << endl;
       out << "" << endl;
       out << "    self.sss.init(\"tray\")" << endl;
       out << "    self.sss.init(\"torso\")" << endl;
       out << "    self.sss.init(\"arm\")" << endl;
       out << "" << endl;
    }

    out << "# get a DB connection" << endl;
    out << "" << endl;
    out << "    if not self.sss.parse:" << endl;
    out << "       try:" << endl;
    out << "" << endl;
    out << "          self.conn = MySQLdb.connect(server_config['mysql_log_server']," << endl;
    out << "                                      server_config['mysql_log_user']," << endl;
    out << "                                      server_config['mysql_log_password']," << endl;
    out << "                                      server_config['mysql_log_db'])" << endl;
    out << "" << endl;
    out << "       except MySQLdb.Error, e:" << endl;
    out << "          print \"Error %d: %s\" % (e.args[0], e.args[1])" << endl;
    out << "          sys.exit (1)" << endl;
    out << "" << endl;
    out << "       rospy.loginfo('MySQL  initialized')" << endl;
    out << "" << endl;

    // run method

    out << "#--------------------------------------------------------------------------------" << endl;
    out << "  def Run(self):" << endl;

    if (localRuleCount > 0)
    {
       out << "" << endl;
       out << "    overallresult=checkRules(self);        # check the rule set" << endl;
       out << "" << endl;
    }
    else
    {
        out << "" << endl;
        out << "# No rules found!" << endl;
        out << "" << endl;
        out << "    overallresult=True;        # no rules thus execute all actions" << endl;
        out << "" << endl;
    }
    // Image history




    if (imageHistory == QMessageBox::Yes)
    {
        out << "" << endl;
        out << "    if not self.sss.parse:" << endl;
        out << "       if (overallresult):" << endl;
        out << "          image = history.ActionHistory()" << endl;
        out << "          image.addPollingHistory('" << sequenceName <<"',5.0)" << endl;
        out << "" << endl;

    }

    bool generateGUICode = false;
    QString GUIScripts = "";

    while(query.next())
    {
   //      qDebug()<< query.value(5).toString();

         if (query.value(2).toString() == "A")
         {


            out << "    if not self.sss.parse:" << endl;
            out <<"       rospy.loginfo(\"" << query.value(5).toString() << "\")" << endl;
            out << "" << endl;
            out << "    if (overallresult):" << endl;

            QString actionString = query.value(7).toString();

       //     qDebug()<< query.value(7).toString();

            QString action  = actionString.section(',',0,0);
            QString robot   = actionString.section(',',1,1);
            QString param1  = actionString.section(',',2,2);
            QString param2  = actionString.section(',',3,3);
            QString wait    = actionString.section(',',4,4);

            if (robot != "0")
            {
               QMessageBox msgBox;
               msgBox.setIcon(QMessageBox::Warning);

               msgBox.setText("Can only generate python for Care-o-Bot at present!");
               msgBox.exec();
               return;
            }
            
            if (action == "base")
            {
                param1.replace(":",",");

                if (wait =="")
                {
                   out <<"       self.sss.move(\"base\"," << param1 << ",False)" << endl;
                }
                else
                {
                   out <<"       self.sss.move(\"base\"," << param1 << ",True)" << endl;
                }

            }
            

            if (action == "torso")
            {
                if (wait =="")
                {
                   out <<"       self.sss.move(\"torso\",\"" << param1 << "\",False)" << endl;
                }
                else
                {
                   out <<"       self.sss.move(\"torso\",\"" << param1 << "\",True)" << endl;
                }
                if (param2 != "")
                {

                    if (wait =="")
                    {
                      out <<"       self.sss.move(\"torso\",\"" << param2 << "\"),False" << endl;
                    }
                    else
                    {
                       out <<"       self.sss.move(\"torso\",\"" << param2 << "\",True)" << endl;
                    }
                }

            }

            if (action == "tray")
            {
                if (wait =="")
                {
                   out <<"       self.sss.move(\"tray\",\"" << param1 << "\",False)" << endl;
                }
                else
                {
                   out <<"       self.sss.move(\"tray\",\"" << param1 << "\",True)" << endl;
                }

            }

            if (action == "light")
            {
              out <<"       self.sss.set_light(\"" << param1 << "\")" << endl;
            }

            if (action == "speak")
            {
              out <<"       self.sss.say([\"" << param1 << "\"])" << endl;
            }

            if (action == "sequence")
            {

              QString fullFN = dir + param1 + ".py";

              out <<"       proc = subprocess.Popen(\"" << fullFN << "\",shell=True, stderr=subprocess.PIPE)" << endl;
              out <<"       return_code = proc.wait()" << endl;
              out <<"       for line in proc.stderr:" << endl;
              out <<"          if (line.rstrip() == \"Failure\"):" << endl;
              out <<"              overallresult = False" << endl;

            }

            if (action == "sleep")
            {
               out <<"       self.sss.sleep(" << param1 << ")" << endl;
            }

            if (action == "GUI")
            {
               generateGUICode = true;
               GUIScripts = param1;

               out <<"       displayGUI(self)" << endl;
            }


            if (action == "cond")
            {

                out << "       if not self.sss.parse:" << endl;
                out << "          cursorSequence = self.conn.cursor()" << endl;
                QString TF = "true";
                if (param2 == "0") TF = "false";

                out << "          sqlSequence = \"INSERT INTO SensorLog (timestamp,sensorId,room,channel,value,status) VALUES (NOW()," << param1 << "\\" << endl;
                out << "               ,'','','" << param2 << "','" << TF << "')\"" << endl;                                                                                                       500,
                out << "          cursorSequence.execute(sqlSequence)" << endl;

            }

            out << "    else:" << endl;
            out << "      if not self.sss.parse:" << endl;
            out <<"         rospy.loginfo(\"Action not executed as rule is invalid!\")" << endl;
            out << "" << endl;

            out << "    if not self.sss.parse:" << endl;
            out << "      if (overallresult):" << endl;
            out << "        sys.stderr.write('Success\\n')" << endl;
            out << "      else:" << endl;
            out << "        sys.stderr.write('Failure\\n')" << endl;
            out << "" << endl;

         //   if (imageHistory == QMessageBox::Yes)
         //   {
         //       out << "" << endl;
         //       out << "    if not self.sss.parse:" << endl;
          //      out << "       if (overallresult):" << endl;
         //       out << "          image.addHistoryAsync('" << sequenceName <<"')" << endl;
          //      out << "" << endl;
//
          //  }

         }


    }

    if (imageHistory == QMessageBox::Yes)
    {
       out << "" << endl;
       out << "    if not self.sss.parse:" << endl;
       out << "          image.cancelPollingHistory('" << sequenceName <<"')" << endl;
       out << "" << endl;

    }

    if (localRuleCount == 0)
    {
       goto checkGUI;
    }

    // rule checker

    out << "" << endl;
    out << "#--------------------------------------------------------------------------------" << endl;
    out << "def checkRules(self):" << endl;
    out << "" << endl;
    out << "" << endl;
    out << "   # this queries the database rules, executes each and veryfies final result" << endl;
    out << "" << endl;
    out << "    overallresult = False    # assume the rule set doesn't apply" << endl;
    out << "" << endl;
    out << "   # this extracts the rules set for this script from the database and checks each rule in turn" << endl;
    out << "   # the rules are and'ed or or'ed based on the rule set" << endl;
    out << "" << endl;
    out << "    if not self.sss.parse:" << endl;
    out << "" << endl;
    out << "         cursorSequence = self.conn.cursor()" << endl;
    out << "" << endl;


    out << "         sqlSequence = \"SELECT   *  FROM  ActionRules\\" << endl;
    out << "                          WHERE     name = \'" << sequenceName << "\'\\"<< endl;
    out << "                            AND     ruleType = \'R\'\\" << endl;
    out << "                          ORDER BY  ruleOrder\"" << endl;


    out << "" << endl;
    out << "         # get the sequence from the sql query" << endl;
    out << "" << endl;
    out << "         cursorSequence.execute(sqlSequence)" << endl;
    out << "" << endl;
    out << "         sequenceRows = cursorSequence.fetchall()" << endl;
    out << "" << endl;
    out << "         # for each row execute the rule to see it is valid" << endl;
    out << "" << endl;
    out << "         recCount = 0" << endl;
    out << "" << endl;
    out << "         try:" << endl;
    out << "           for sequenceRow in sequenceRows:" << endl;
    out << "" << endl;
    out << "              recCount = recCount + 1" << endl;
    out << "" << endl;
    out << "              ANDORSwitch = \"\"" << endl;
    out << "" << endl;
    out << "              andOr = sequenceRow[4]" << endl;
    out << "" << endl;
    out << "              if andOr== 1:" << endl;
    out << "                 ANDORSwitch = \"AND\"" << endl;
    out << "" << endl;
    out << "              if andOr== 2:" << endl;
    out << "                 ANDORSwitch = \"OR\"" << endl;
    out << "" << endl;
    out << "              rospy.loginfo(\"%s %s\",sequenceRow[5],ANDORSwitch)" << endl;
    out << "" << endl;
    out << "              # now do each rule" << endl;
    out << "" << endl;
    out << "              cursorRule = self.conn.cursor()" << endl;
    out << "" << endl;
    out << "              sqlRule = sequenceRow[6];" << endl;
    out << "" << endl;
    out << "              cursorRule.execute(sqlRule)" << endl;
    out << "" << endl;
    out << "              ruleRows = cursorRule.fetchone()" << endl;
    out << "" << endl;
    out << "              # returning data means that the rule is true" << endl;
    out << "" << endl;
    out << "              if (ruleRows==None):" << endl;
    out << "                 rospy.loginfo(\"%s FALSE\",sqlRule)" << endl;
    out << "                 result = False" << endl;
    out << "              else:" << endl;
    out << "                 rospy.loginfo(\"%s TRUE\",sqlRule)" << endl;
    out << "                 result = True " << endl;
    out << "" << endl;
    out << "              if (recCount == 1): " << endl;
    out << "                 overallresult = result" << endl;
    out << "                 prevANDORSwitch = ANDORSwitch" << endl;
    out << "              else:                                  # now AND or OR each row in turn" << endl;
    out << "                 if (prevANDORSwitch == \"OR\"):" << endl;
    out << "                    overallresult = overallresult or result" << endl;
    out << "                 if (prevANDORSwitch == \"AND\"):" << endl;
    out << "                    overallresult = overallresult and result" << endl;
    out << "                 if (prevANDORSwitch == \"\"):" << endl;
    out << "                    overallresult = overallresult and result;" << endl;
    out << "" << endl;
    out << "                 prevANDORSwitch = ANDORSwitch" << endl;
    out << "" << endl;
    out << "           if (overallresult):                      # final result" << endl;
    out << "             rospy.loginfo(\"Total Rule Set is VALID! :) \")" << endl;
    out << "           else:" << endl;
    out << "             rospy.loginfo(\"Total Rule Set is INVALID :( \")" << endl;
    out << "" << endl;
    out << "         except MySQLdb.Error, e:" << endl;
    out << "           rospy.loginfo(\"Error %d: %s\" % (e.args[0],e.args[1]))" << endl;
    out << "           sys.exit(1)  " << endl;
    out << "" << endl;
    out << "         finally:   " << endl;
    out << "           cursorSequence.close() " << endl;
    out << "" << endl;
    out << "         return overallresult" << endl;
    out << "" << endl;

    checkGUI:

    // User Interface

    QString script1, script2,script3,script4;
    QString fullFN1, fullFN2, fullFN3, fullFN4;

    if (!generateGUICode)
    {
        goto mainCall;
    }

    out << "" << endl;
    out << "#--------------------------------------------------------------------------------" << endl;
    out << "def displayGUI(self):" << endl;
    out << "" << endl;
    out << "   # this sets up the database for GUI display and waits for a result" << endl;
    out << "   # It then executes a script depending on user response" << endl;
    out << "" << endl;
    out << "   if not self.sss.parse:" << endl;
    out << "" << endl;
    out << "      cursorSequence = self.conn.cursor()" << endl;
    out << "" << endl;
    out << "      sqlSequence = \"UPDATE userInterfaceGUI SET guiMsgResult = NULL\\" << endl;
    out << "                      WHERE name = '" << sequenceName << "'\"" << endl;
    out << "" << endl;
    out << "      try:" << endl;
    out << "         cursorSequence.execute(sqlSequence)  # get the sequence from the sql query  " << endl;
    out << "         self.conn.commit()                   # Commit  changes in the database" << endl;
    out << "" << endl;
    out << "      except MySQLdb.Error, e:" << endl;
    out << "         rospy.loginfo(\"Error %d: %s\" % (e.args[0],e.args[1]))" << endl;
    out << "         self.rollback()" << endl;
    out << "         self.conn.close()" << endl;
    out << "         sys.exit(1)" << endl;
    out << "" << endl;
    out << "      awaitingUserResponse = True" << endl;
    out << "      resultFromUser = 0" << endl;
    out << "      numSeconds = 0" << endl;
    out << "" << endl;
    out << "      rospy.loginfo(\"Waiting for user response...\")" << endl;
    out << "" << endl;
    out << "      while awaitingUserResponse:" << endl;
    out << "" << endl;
    out << "         time.sleep(1)" << endl;
    out << "" << endl;
    out << "         numSeconds = numSeconds + 1" << endl;
    out << "" << endl;
    out << "         if (numSeconds > 60):" << endl;
    out << "            rospy.loginfo(\"Error: waited too long for user response - exiting!\")" << endl;
    out << "            sys.exit(1)  " << endl;
    out << "" << endl;
    out << "         sqlSequence = \"SELECT * FROM  userInterfaceGUI WHERE name = '" << sequenceName << "'\"" << endl;
    out << "         cursorSequence.execute(sqlSequence)      # get the sequence from the sql query" << endl;
    out << "         sequenceRows = cursorSequence.fetchall()" << endl;
    out << "" << endl;
    out << "         try:" << endl;
    out << "" << endl;
    out << "          for sequenceRow in sequenceRows:      # for each row execute the rule to see it is valid " << endl;
    out << "" << endl;
    out << "            resultFromUser = sequenceRow[13]" << endl;
    out << "" << endl;
    out << "            if (resultFromUser == None): # no response" << endl;
    out << "               break" << endl;
    out << "" << endl;
    out << "            awaitingUserResponse = False" << endl;
    out << "" << endl;
    out << "            rospy.loginfo(\"User entered %s\",resultFromUser)" << endl;
    out << "" << endl;

    script1 = GUIScripts.section("@",0,0);
    script2 = GUIScripts.section("@",1,1);
    script3 = GUIScripts.section("@",2,2);
    script4 = GUIScripts.section("@",3,3);

  //  qDebug()<< script1 <<" "<< script2 << " " << script3<< " " << script4;

    fullFN1 = dir + script1 + ".py";
    fullFN2 = dir + script2 + ".py";
    fullFN3 = dir + script3 + ".py";
    fullFN4 = dir + script4 + ".py";


     if (script1 != "")
     {
        out << "            if (resultFromUser == 1):" << endl;
        out <<"                proc = subprocess.Popen(\"" << fullFN1 << "\",shell=True, stderr=subprocess.PIPE)" << endl;
        out <<"                return_code = proc.wait()" << endl;
        out <<"                for line in proc.stderr:" << endl;
        out <<"                  if (line.rstrip() == \"Failure\"):" << endl;
        out <<"                     overallresult = False" << endl;
        out << "" << endl;
    }

    if (script2 != "")
    {
        out << "            if (resultFromUser == 2):" << endl;
        out <<"                proc = subprocess.Popen(\"" << fullFN2 << "\",shell=True, stderr=subprocess.PIPE)" << endl;
        out <<"                return_code = proc.wait()" << endl;
        out <<"                for line in proc.stderr:" << endl;
        out <<"                  if (line.rstrip() == \"Failure\"):" << endl;
        out <<"                     overallresult = False" << endl;
        out << "" << endl;
    }

    if (script3 != "")
    {
        out << "            if (resultFromUser == 3):" << endl;
        out <<"                proc = subprocess.Popen(\"" << fullFN3 << "\",shell=True, stderr=subprocess.PIPE)" << endl;
        out <<"                return_code = proc.wait()" << endl;
        out <<"                for line in proc.stderr:" << endl;
        out <<"                  if (line.rstrip() == \"Failure\"):" << endl;
        out <<"                     overallresult = False" << endl;
        out << "" << endl;
    }

    if (script4 != "")
    {
        out << "            if (resultFromUser == 4):" << endl;
        out <<"                proc = subprocess.Popen(\"" << fullFN4 << "\",shell=True, stderr=subprocess.PIPE)" << endl;
        out <<"                return_code = proc.wait()" << endl;
        out <<"                for line in proc.stderr:" << endl;
        out <<"                  if (line.rstrip() == \"Failure\"):" << endl;
        out <<"                     overallresult = False" << endl;
        out << "" << endl;
    }

    out << "         except MySQLdb.Error, e:" << endl;
    out << "            rospy.loginfo(\"Error %d: %s\" % (e.args[0],e.args[1]))" << endl;
    out << "            sys.exit(1)" << endl;

    mainCall:

// Main call

    out << "#--------------------------------------------------------------------------------" << endl;
    out << "if __name__ == \"__main__\":" << endl;
    out << "   SCRIPT = "<< sequenceName << "()" << endl;
    out << "   SCRIPT.Start()" << endl;


    out << "#--------------------------------------------------------------------------------" << endl;



}

void MainWindow::on_QBpushButton_clicked()
{

    QString sequenceName = ui->SeqComboBox->currentText();
    QSqlQuery query;
    QSqlQuery queryLogic;

    query.clear();

    query.prepare("SELECT * FROM  ActionRules WHERE name = :name ORDER BY ruleOrder");

    query.bindValue(":name",sequenceName );

    if (!query.exec())
    {
        qDebug() << "Can't select from actionrules table!" << query.executedQuery();
        return;
    }


    QString prevANDORSwitch = "undefined";
    QString ANDORSwitch;

    int recCount=0;
    bool overallresult, result;

    while(query.next())
    {
         if (query.value(2).toString() != "R")
         {
             return;
         }

         recCount++;

         qDebug()<<query.value(4).toString()<<"  "<<query.value(6).toString();

         QString ANDORSwitch = "undefined";

         if (query.value(4).toInt() == 1)
         {
            ANDORSwitch = "AND";
         }

         if (query.value(4).toInt() == 2)
         {
            ANDORSwitch = "OR";
         }

         queryLogic.clear();

         queryLogic.prepare(query.value(6).toString());

         if (!queryLogic.exec())
         {
             qDebug() << "Can't execute rule from actionrules table!" << queryLogic.executedQuery();
             return;
         }

         if(queryLogic.size()==1)
         {
             result = true;
             qDebug()<<" <-- true";
         }
         else
         {
             result = false;
             qDebug()<<" <-- false";
         }

         if (recCount == 1)             // first record
         {
            overallresult = result;
            prevANDORSwitch = ANDORSwitch;

         }
         else
         {
             if (prevANDORSwitch == "OR")
             {
                 overallresult = overallresult || result;
             }

             if (prevANDORSwitch == "AND")
             {
                 overallresult = overallresult && result;
             }

             if (prevANDORSwitch == "undefined")
             {
                 overallresult = overallresult && result;
             }
             prevANDORSwitch = ANDORSwitch;

         }

     }

     if (overallresult)
     {
        qDebug()<<"Rule is valid!";
     }
     else
     {
        qDebug()<<"Rule is invalid!";
     }


}


void MainWindow::on_BigCupboardDoorBottomCheckBox_toggled(bool checked)
{
    {

        ui->BCDBOpenRadioButton->setEnabled(checked);
        ui->BCDBClosedRadioButton->setEnabled(checked);
        ui->BCDBANDRadioButton->setEnabled(checked);
        ui->BCDBORRadioButton->setEnabled(checked);

        if (checked)
        {
           ruleCount++;
        }
        else
        {
            ruleCount--;
        }

    }
}

void MainWindow::on_BigCupboardDoorTopCheckBox_toggled(bool checked)
{
    ui->BCDTOpenRadioButton->setEnabled(checked);
    ui->BCDTClosedRadioButton->setEnabled(checked);
    ui->BCDTANDRadioButton->setEnabled(checked);
    ui->BCDTORRadioButton->setEnabled(checked);

    if (checked)
    {
       ruleCount++;
    }
    else
    {
        ruleCount--;
    }

}


void MainWindow::on_SmallCupboardDoorLeftCheckBox_toggled(bool checked)
{
    ui->SCDLOpenRadioButton->setEnabled(checked);
    ui->SCDLClosedRadioButton->setEnabled(checked);
    ui->SCDLANDRadioButton->setEnabled(checked);
    ui->SCDLORRadioButton->setEnabled(checked);

    if (checked)
    {
       ruleCount++;
    }
    else
    {
        ruleCount--;
    }
}



void MainWindow::on_SmallCupboardDooRightCheckBox_toggled(bool checked)
{
    ui->SCDROpenRadioButton->setEnabled(checked);
    ui->SCDRClosedRadioButton->setEnabled(checked);
    ui->SCDRANDRadioButton->setEnabled(checked);
    ui->SCDRORRadioButton->setEnabled(checked);

    if (checked)
    {
       ruleCount++;
    }
    else
    {
        ruleCount--;
    }
}

void MainWindow::on_SmallCupboardDrawBottomCheckBox_toggled(bool checked)
{
    ui->SCDBOpenRadioButton->setEnabled(checked);
    ui->SCDBClosedRadioButton->setEnabled(checked);
    ui->SCDBANDRadioButton->setEnabled(checked);
    ui->SCDBORRadioButton->setEnabled(checked);

    if (checked)
    {
       ruleCount++;
    }
    else
    {
        ruleCount--;
    }
}

void MainWindow::on_SmallCupboardDoorMiddleCheckBox_toggled(bool checked)
{
    ui->SCDMOpenRadioButton->setEnabled(checked);
    ui->SCDMClosedRadioButton->setEnabled(checked);
    ui->SCDMANDRadioButton->setEnabled(checked);
    ui->SCDMORRadioButton->setEnabled(checked);

    if (checked)
    {
       ruleCount++;
    }
    else
    {
        ruleCount--;
    }
}

void MainWindow::on_SmallCupboardDoorTopCheckBox_toggled(bool checked)
{
    ui->SCDTOpenRadioButton->setEnabled(checked);
    ui->SCDTClosedRadioButton->setEnabled(checked);
    ui->SCDTANDRadioButton->setEnabled(checked);
    ui->SCDTORRadioButton->setEnabled(checked);

    if (checked)
    {
       ruleCount++;
    }
    else
    {
        ruleCount--;
    }
}

void MainWindow::on_scheduleCheckBox_clicked()
{
    updateSequenceTable();

}

void MainWindow::on_moveRobotComboBox_currentIndexChanged(QString str )
{

    QSqlQuery query("SELECT locationId, orientation FROM Locations where locationId = " + str.section("::", 1, 1));

    int rows = 0;
    while(query.next())
    {
        rows++;

        if ( query.value(0).toString() != ui->moveRobotComboBox->currentText().section("::", 1, 1) || rows > 1)
        {
            QMessageBox msgBox;
            msgBox.setIcon(QMessageBox::Warning);

            msgBox.setText("Error on query to location table!");
            msgBox.exec();
            return;
        }
        else
        {
            ui->moveRobotSpinBox->setValue(query.value(1).toInt());
        }


    }

}


void MainWindow::on_robotGUI_toggled(bool checked)
{
    ui->robotGUIgroupBox->setEnabled(checked);

    ui->GUI1LargeCheckBox->setEnabled(false);
    ui->GUI2LargeCheckBox->setEnabled(false);
    ui->GUI3LargeCheckBox->setEnabled(false);
    ui->GUI4LargeCheckBox->setEnabled(false);

    QString userStr = ui->userLocationUserComboBox->currentText();

    int userId;

    if (userStr =="")
    {
        userId=1; //defaults to first user who is english
    }
    else
    {
        userId = userStr.section("::",1,1).toInt();
    }
    qDebug()<<userStr;
    qDebug()<<userId;

    QString userIdStr;
    userIdStr.setNum(userId);

    QString Qry = "SELECT M.message FROM Messages M, Users U where M.actionMsg = 1 and  M.languageId = U.languageId and U.userId = " + userIdStr;

    qDebug()<<Qry;

    if (checked)
    {
        QSqlQuery query(Qry);

        ui->GUI1ComboBox->clear();
        ui->GUI2ComboBox->clear();
        ui->GUI3ComboBox->clear();
        ui->GUI4ComboBox->clear();

        while(query.next())
        {
            ui->GUI1ComboBox->addItem(query.value(0).toString());
            ui->GUI2ComboBox->addItem(query.value(0).toString());
            ui->GUI3ComboBox->addItem(query.value(0).toString());
            ui->GUI4ComboBox->addItem(query.value(0).toString());
        }

        ui->GUI1ComboBox->clearEditText();
        ui->GUI2ComboBox->clearEditText();
        ui->GUI3ComboBox->clearEditText();
        ui->GUI4ComboBox->clearEditText();

        actionCount++;
    }
    else
    {
        actionCount--;
    }



    if (checked)
    {

        ui->GUI1ExecuteComboBox->clear();
        ui->GUI2ExecuteComboBox->clear();
        ui->GUI3ExecuteComboBox->clear();
        ui->GUI4ExecuteComboBox->clear();

        Qry = "SELECT * FROM userInterfaceGUI WHERE name = \"" + ui->SeqComboBox->currentText() + "\"";

        qDebug()<<Qry;

        QSqlQuery query(Qry);

        while(query.next())
        {
            ui->GUI1ComboBox->setEditText(query.value(1).toString());
            ui->GUI2ComboBox->setEditText(query.value(4).toString());
            ui->GUI3ComboBox->setEditText(query.value(7).toString());
            ui->GUI4ComboBox->setEditText(query.value(10).toString());

            ui->GUI1LargeCheckBox->setChecked(query.value(2).toInt());
            ui->GUI2LargeCheckBox->setChecked(query.value(5).toInt());
            ui->GUI3LargeCheckBox->setChecked(query.value(8).toInt());
            ui->GUI4LargeCheckBox->setChecked(query.value(11).toInt());

            ui->GUI1ExecuteComboBox->insertItem(0,query.value(3).toString());
            ui->GUI2ExecuteComboBox->insertItem(0,query.value(6).toString());
            ui->GUI3ExecuteComboBox->insertItem(0,query.value(9).toString());
            ui->GUI4ExecuteComboBox->insertItem(0,query.value(12).toString());

        }


    }

    if (checked)
    {
        QString seqQuery;
        QString locn;
        locn.setNum(experimentLocation);
        seqQuery = "SELECT * FROM Sequences WHERE experimentalLocationId = " + locn;

        QSqlQuery query(seqQuery);

        while(query.next())
        {
           ui->GUI1ExecuteComboBox->addItem(query.value(0).toString());
           ui->GUI2ExecuteComboBox->addItem(query.value(0).toString());
           ui->GUI3ExecuteComboBox->addItem(query.value(0).toString());
           ui->GUI4ExecuteComboBox->addItem(query.value(0).toString());

       }

    }

}


void MainWindow::on_GUI1CheckBox_clicked(bool checked)
{
     ui->GUI1ComboBox->setEnabled(checked);
     ui->GUI1LargeCheckBox->setEnabled(checked);
     ui->GUI1ExecuteComboBox->setEnabled(checked);
 //    if (checked) ui->GUI1ComboBox->clearEditText();
}

void MainWindow::on_GUI2CheckBox_clicked(bool checked)
{
    ui->GUI2ComboBox->setEnabled(checked);
    ui->GUI2LargeCheckBox->setEnabled(checked);
    ui->GUI2ExecuteComboBox->setEnabled(checked);
//    if (checked) ui->GUI2ComboBox->clearEditText();
}

void MainWindow::on_GUI3CheckBox_clicked(bool checked)
{
    ui->GUI3ComboBox->setEnabled(checked);
    ui->GUI3LargeCheckBox->setEnabled(checked);
    ui->GUI3ExecuteComboBox->setEnabled(checked);
//    if (checked) ui->GUI3ComboBox->clearEditText();
}

void MainWindow::on_GUI4CheckBox_clicked(bool checked)
{
    ui->GUI4ComboBox->setEnabled(checked);
    ui->GUI4LargeCheckBox->setEnabled(checked);
    ui->GUI4ExecuteComboBox->setEnabled(checked);
//    if (checked) ui->GUI4ComboBox->clearEditText();
}




void MainWindow::on_Goal1CheckBox_toggled(bool checked)
{
    ui->Goal1TRUERadioButton->setEnabled(checked);
    ui->Goal1FALSERadioButton->setEnabled(checked);
    ui->Goal1ANDRadioButton->setEnabled(checked);
    ui->Goal1ORRadioButton->setEnabled(checked);
    ui->Goal1ComboBox->setEnabled(checked);
    ui->Goal1LineEdit->setEnabled(checked);

    qDebug()<<ruleCount;

    if (checked)
    {
       ruleCount++;

       QSqlQuery query("SELECT * FROM Sensors where sensorTypeId = 6  OR sensorTypeId = 9");

    //   query.exec();

       ui->Goal1ComboBox->clear();

       while(query.next())
       {
          ui->Goal1ComboBox->addItem("::" + query.value(0).toString() + ":: " + query.value(3).toString());
       }

    }
    else
    {
        ruleCount--;
    }


}

void MainWindow::on_robotGoal1CheckBox_toggled(bool checked)
{
    ui->robotGoal1ComboBox->setEnabled(checked);
    ui->robotGoal1LineEdit->setEnabled(checked);

    if (checked)
    {
        QSqlQuery query("SELECT * FROM Sensors where sensorTypeId = 6");

        query.exec();

        ui->robotGoal1ComboBox->clear();

        while(query.next())
        {
           ui->robotGoal1ComboBox->addItem("::"+ query.value(0).toString() + "::" + query.value(3).toString());
        }

        actionCount++;
    }
    else
    {
        actionCount--;

    }
}

void MainWindow::on_robotEyesCheckBox_toggled(bool checked)
{
    ui->robotEyesGroupBox->setEnabled(checked);

    if (checked)
    {
       actionCount++;
    }
    else
    {
        actionCount--;
    }

}

void MainWindow::on_condAddRuleButton_clicked()
{
    if (ui->condLineEdit->text().isEmpty())
    {
        QMessageBox msgBox;
        msgBox.setIcon(QMessageBox::Warning);

        msgBox.setText("You need to enter a condition/goal name!");
        msgBox.exec();
        return;
    }

    QSqlQuery query;

    query.prepare("SELECT MAX(sensorId) FROM Accompany.Sensors where sensorId >499 and sensorId < 600 ");

    if (!query.exec())
    {
        qDebug() << query.lastQuery();

        QMessageBox msgBox;
        msgBox.setIcon(QMessageBox::Critical);

        msgBox.setText("Can't add/update Sequence table - duplicate?");
        msgBox.exec();

        qCritical("Cannot add/update: %s (%s)",
                  db.lastError().text().toLatin1().data(),
                  qt_error_string().toLocal8Bit().data());
        return;
    }

    int sId;

    while(query.next())
    {

        if (query.value(0).toInt() == 0)
        {
            sId = 500;
        }
        else
        {
           sId = query.value(0).toInt() + 1;
        }

        qDebug() << sId;

        query.prepare("INSERT INTO Sensors VALUES (:sensorId, '0', '0', :name, '5', 'Predicate', 'N/A', '6',NOW(),NOW(),0,'false','false')");

       query.bindValue(":sensorId",sId);
       query.bindValue(":name",ui->condLineEdit->text());


       if (!query.exec())
       {

        qDebug() << query.lastQuery();

        QMessageBox msgBox;
        msgBox.setIcon(QMessageBox::Critical);

        msgBox.setText("Can't add to Sensors table - duplicate?");
        msgBox.exec();

        qCritical("Cannot add/update: %s (%s)",
                  db.lastError().text().toLatin1().data(),
                  qt_error_string().toLocal8Bit().data());
        return;
       }

       ui->condLineEdit->clear();
    }

}

void MainWindow::on_seqDescLineEdit_editingFinished()
{
    updateSequenceTable();
}

void MainWindow::on_seqTypeComboBox_currentIndexChanged(int index)
{
      updateSequenceTable();
}

void MainWindow::on_Goal1ComboBox_currentIndexChanged(QString condition)
{

    if (condition.section("::",1,1).toInt() < 900)
    {
        ui->Goal1LineEdit->setEnabled(true);             // predicates can take a parameter
        ui->Goal1AndOrGroupBox->setEnabled(false);
    }
    else
    {
        ui->Goal1LineEdit->setEnabled(false);            // contexts can be true/false
        ui->Goal1AndOrGroupBox->setEnabled(true);
    }
}

void MainWindow::on_HUYTlivingSofa1CheckBox_toggled(bool checked)
{
    ui->HUYTLRS1OccupiedRadioButton->setEnabled(checked);
    ui->HUYTLRS1NotOccupiedRadioButton->setEnabled(checked);
    ui->HUYTLRS1ANDRadioButton->setEnabled(checked);
    ui->HUYTLRS1ORRadioButton->setEnabled(checked);

    if (checked)
    {
       ruleCount++;
    }
    else
    {
        ruleCount--;
    }

}

void MainWindow::on_cupLevelCheckBox_toggled(bool checked)
{
    ui->HUYTCupEmptyRadioButton->setEnabled(checked);
    ui->HUYTCupFullRadioButton->setEnabled(checked);
    ui->cupLevelANDRadioButton->setEnabled(checked);
    ui->cupLevelORRadioButton->setEnabled(checked);

    if (checked)
    {
       ruleCount++;
    }
    else
    {
        ruleCount--;
    }
}

void MainWindow::on_ZUYDDoorbellCheckBox_toggled(bool checked)
{
    ui->ZUYDDoorbellOnRadioButton->setEnabled(checked);
    ui->ZUYDDoorbellOffRadioButton->setEnabled(checked);
    ui->ZUYDDoorbellANDRadioButton->setEnabled(checked);
    ui->ZUYDDoorbellORRadioButton->setEnabled(checked);

    if (checked)
    {
       ruleCount++;
    }
    else
    {
        ruleCount--;
    }
}

void MainWindow::on_ZUYDFridgeCheckBox_toggled(bool checked)
{
    ui->ZUYDFridgeOnRadioButton->setEnabled(checked);
    ui->ZUYDFridgeOffRadioButton->setEnabled(checked);
    ui->ZUYDFridgeANDRadioButton->setEnabled(checked);
    ui->ZUYDFridgeORRadioButton->setEnabled(checked);

    if (checked)
    {
       ruleCount++;
    }
    else
    {
        ruleCount--;
    }
}

void MainWindow::on_robotExpressionCheckBox_toggled(bool checked)
{

        ui->expressionGroupBox->setEnabled(checked);

        if (checked)
        {
            actionCount++;
        }
        else
        {
            actionCount--;
        }

}

void MainWindow::on_HUYTlivingSofa2CheckBox_toggled(bool checked)
{
    ui->HUYTLRS2OccupiedRadioButton->setEnabled(checked);
    ui->HUYTLRS2NotOccupiedRadioButton->setEnabled(checked);
    ui->HUYTLRS2ANDRadioButton->setEnabled(checked);
    ui->HUYTLRS2ORRadioButton->setEnabled(checked);

    if (checked)
    {
       ruleCount++;
    }
    else
    {
        ruleCount--;
    }
}

void MainWindow::on_HUYTlivingSofa3CheckBox_toggled(bool checked)
{
    ui->HUYTLRS3OccupiedRadioButton->setEnabled(checked);
    ui->HUYTLRS3NotOccupiedRadioButton->setEnabled(checked);
    ui->HUYTLRS3ANDRadioButton->setEnabled(checked);
    ui->HUYTLRS3ORRadioButton->setEnabled(checked);

    if (checked)
    {
       ruleCount++;
    }
    else
    {
        ruleCount--;
    }
}

void MainWindow::on_armCheckBox_toggled(bool checked)
{
    ui->robotArmGroupBox->setEnabled(checked);

    if (checked)
    {
       actionCount++;
    }
    else
    {
        actionCount--;
    }
}

void MainWindow::on_apCheckBox_toggled(bool checked)
{
    ui->apGroupBox->setEnabled(checked);

    if (checked)
    {
        QString userStr = ui->userLocationUserComboBox->currentText();

        QString userId;

        if (userStr =="")
        {
            userId=1; //defaults to first user who is english
        }
        else
        {
            userId = userStr.section("::",1,1);
        }

        QSqlQuery query;

        query.clear();

        query.prepare("SELECT ap.apid, m.message \
                         FROM ActionPossibilities ap,\
                              Messages m,\
                              Users u\
                      WHERE ap.ap_Text  = m.messageId\
                        AND m.languageId = u.languageID\
                        AND u.userId = " + userId);

        query.exec();

        ui->apComboBox->clear();

        while(query.next())
        {
            ui->apComboBox->addItem("::"+ query.value(0).toString() + "::" + query.value(1).toString());
        }

       actionCount++;
    }
    else
    {
        actionCount--;
    }
}
