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
#include <phonon/audiooutput.h>
#include <phonon/mediaobject.h>

#include <QSqlRelationalTableModel>
QSqlDatabase db;
bool dbOpen;
bool firstTime;
int experimentLocation;
QString activeUser;
int activeRobot;

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    closeDownRequest=false;
}

void MainWindow::openbDB()
{
    QString host, user, pw, dBase;
    bool ok;

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

    ui->runningAtLabel->setText(lv);



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

}

void MainWindow::setup()
 {

    QSqlQuery query("SELECT *  FROM SessionControl WHERE SessionId = 1 LIMIT 1");

    if (query.next())
    {
       activeUser = query.value(5).toString();
       experimentLocation = query.value(6).toInt();

    }

    QString expL;
    expL.setNum(experimentLocation);

    QString qry = "SELECT activeRobot FROM ExperimentalLocation WHERE id =";
    qry += expL;

    query.clear();
    query = qry;

    query.exec();

    if (query.next())
    {
       activeRobot = query.value(0).toInt();
    }

    qDebug()<<qry;

    getLocations();
//    QCoreApplication app(argc, argv);
//    app.setApplicationName("SensorSim");

}

void MainWindow::getLocations()
{
    firstTime = true;

    QString locQuery;
    QSqlQuery query;

    locQuery = "SELECT locationId FROM Users where userId = " + activeUser + " LIMIT 1";

    query = locQuery;

    query.exec();

    QString loc;
    while(query.next())
    {
       loc = query.value(0).toString();
    }

    locQuery = "SELECT  L1.locationId, L1.where, L2.where, L1.name, IF(STRCMP(L1.name,L2.name),L2.name,''), IF(STRCMP(L2.name,L3.name),L3.name,''), IF(STRCMP(L3.name,L4.name),L4.name,'')\
               FROM Locations L1,\
                    Locations L2,\
                    Locations L3,\
                    Locations L4\
               WHERE L2.locationId = L1.where\
                 AND L3.locationId = L2.where\
                 AND L4.locationId = L3.where\
                 AND L1.validUserLocation = 1";

     locQuery += " AND ((L1.locationId < 100) or L1.locationId = 999) ORDER BY L1.locationId";

     query = locQuery;

    ui->userLocationComboBox->clear();

    QString q1, q2, q3;

    q1 = q2 = q3 = "";

    int index = 0;
    int dispIndex = -1;

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

        if (query.value(0).toString() == loc)
        {
            dispIndex = index;
        }
        index++;
    }

   if ( dispIndex != -1 )
   { // -1 for not found
          ui->userLocationComboBox->setCurrentIndex(dispIndex);
   }


   // robot

   QString actR;
   actR.setNum(activeRobot);

   locQuery = "SELECT locationId FROM Robot where robotId = ";
   locQuery += actR + " LIMIT 1";

   query = locQuery;

   query.exec();

   while(query.next())
   {
      loc = query.value(0).toString();
  }



   index = 0;
   dispIndex = -1;

   locQuery = "SELECT  L1.locationId, L1.where, L2.where, L1.name, IF(STRCMP(L1.name,L2.name),L2.name,''), IF(STRCMP(L2.name,L3.name),L3.name,''), IF(STRCMP(L3.name,L4.name),L4.name,'')\
              FROM Locations L1,\
                   Locations L2,\
                   Locations L3,\
                   Locations L4\
              WHERE L2.locationId = L1.where\
                AND L3.locationId = L2.where\
                AND L4.locationId = L3.where\
                AND L1.validRobotLocation = 1";


        locQuery += " AND ((L1.locationId < 100) OR L1.locationId = 999) ORDER BY L1.locationId";

   query = locQuery;

   ui->robotLocationComboBox->clear();


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

 //      qDebug()<<loc<<" "<<query.value(0).toString();
       if (query.value(0).toString() == loc)
       {
           dispIndex = index;
       }
       index++;
   }

   if ( dispIndex != -1 )
   { // -1 for not found
          ui->robotLocationComboBox->setCurrentIndex(dispIndex);
   }

   locQuery = "SELECT value from Sensors where sensorId = 15 LIMIT 1";

   query = locQuery;

   query.exec();

   while(query.next())
   {
      if (query.value(0).toInt() == 1)
      {
         ui->sofa1CheckBox->setChecked(false);
      }
      else
      {
         ui->sofa1CheckBox->setChecked(true);
      }
   }

   locQuery = "SELECT value from Sensors where sensorId = 16 LIMIT 1";

   query = locQuery;

   query.exec();

   while(query.next())
   {
      if (query.value(0).toInt() == 1)
      {
         ui->sofa2CheckBox->setChecked(false);
      }
      else
      {
         ui->sofa2CheckBox->setChecked(true);
      }
   }

   locQuery = "SELECT value from Sensors where sensorId = 17 LIMIT 1";
   query.clear();
   query = locQuery;

   query.exec();

   while(query.next())
   {
      if (query.value(0).toInt() == 1)
      {
         ui->sofa3CheckBox->setChecked(false);
      }
      else
      {
         ui->sofa3CheckBox->setChecked(true);
      }
   }


   locQuery = "SELECT value from Sensors where sensorId = 18 LIMIT 1";
   query.clear();
   query = locQuery;

   query.exec();

   while(query.next())
   {
      if (query.value(0).toInt() == 1)
      {
         ui->sofa4CheckBox->setChecked(false);
      }
      else
      {
         ui->sofa4CheckBox->setChecked(true);
      }
   }

   locQuery = "SELECT value from Sensors where sensorId = 19 LIMIT 1";
   query.clear();
   query = locQuery;

   query.exec();

   while(query.next())
   {
      if (query.value(0).toInt() == 1)
      {
         ui->sofa5CheckBox->setChecked(false);
      }
      else
      {
         ui->sofa5CheckBox->setChecked(true);
      }
   }


   locQuery = "SELECT status from Sensors where sensorId = 500 LIMIT 1";

   query = locQuery;

   query.exec();

   while(query.next())
   {
      if (query.value(0).toString() == "Full")
      {
         ui->trayCheckBox->setChecked(true);
      }
      else
      {
         ui->trayCheckBox->setChecked(false);
      }
   }



   locQuery = "SELECT value from Sensors where sensorId = 501 LIMIT 1";

   query = locQuery;

   query.exec();

   while(query.next())
   {
      if (query.value(0).toString() == "Raised")
      {
         ui->trayStatusCheckBox->setChecked(true);
      }
      else
      {
         ui->trayStatusCheckBox->setChecked(false);
      }
   }




   locQuery = "SELECT value from Sensors where sensorId = 60 LIMIT 1";

   query = locQuery;

   query.exec();

   while(query.next())
   {
      ui->cupCheckBox->setChecked(query.value(0).toBool());
   }

   locQuery = "SELECT value from Sensors where sensorId = 50 LIMIT 1";

   query = locQuery;

   query.exec();

   while(query.next())
   {
   //    qDebug()<<query.value(0).toInt();
      if (query.value(0).toInt() == 0)
      {
         ui->fridgeCheckBox->setChecked(true);
      }
      else
      {
         ui->fridgeCheckBox->setChecked(false);
      }
   }

   locQuery = "SELECT status from Sensors where sensorId = 49 LIMIT 1";

   query = locQuery;

   query.exec();

   while(query.next())
   {
       if (query.value(0).toString() == "On")
      {
         ui->TVCheckBox->setChecked(true);
      }
      else
      {
         ui->TVCheckBox->setChecked(false);
      }
   }

   locQuery = "SELECT status from Sensors where sensorId = 51 LIMIT 1";

   query = locQuery;

   query.exec();

   while(query.next())
   {
       if (query.value(0).toString() == "On")
      {
         ui->kettleCheckBox->setChecked(true);
      }
      else
      {
         ui->kettleCheckBox->setChecked(false);
      }
   }

   locQuery = "SELECT status from Sensors where sensorId = 54 LIMIT 1";

   query = locQuery;

   query.exec();

   while(query.next())
   {
       if (query.value(0).toString() == "On")
      {
         ui->microCheckBox->setChecked(true);
      }
      else
      {
         ui->microCheckBox->setChecked(false);
      }
   }

   locQuery = "SELECT status from Sensors where sensorId = 56 LIMIT 1";

   query = locQuery;

   query.exec();

   while(query.next())
   {
       if (query.value(0).toString() == "On")
      {
         ui->toasterCheckBox->setChecked(true);
      }
      else
      {
         ui->toasterCheckBox->setChecked(false);
      }
   }

   firstTime = false;






   firstTime = false;
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

void MainWindow::updateSensorLog(int sensor, int value, QString stat)
{
        QSqlQuery query;

        query.prepare("INSERT INTO SensorLog (timestamp, sensorId, room, channel, value, status )\
                      VALUES(NOW(), :sensorId, :room, :channel, :value, :status) ");


        query.bindValue(":sensorId",sensor);
        query.bindValue(":room","");
        query.bindValue(":channel","");

        if (sensor == 500)                     // becasue 500 is a predicate sensor and takes text values, but ui returns bool
        {
           if (value == 0)
           {
              query.bindValue(":value","Empty");
           }
           else
           {
              query.bindValue(":value","Full");
           }
        }
        else
        {
          query.bindValue(":value",value);

        }

        if (sensor == 501)                     // becasue 501is a predicate sensor and takes text values, but ui returns bool
        {
           if (value == 0)
           {
              query.bindValue(":value","Lowered");
           }
           else
           {
              query.bindValue(":value","Raised");
           }
        }
        else
        {
          query.bindValue(":value",value);

        }



        query.bindValue(":status",stat);

        if (!query.exec())
        {
            QMessageBox msgBox;
            msgBox.setIcon(QMessageBox::Critical);

            msgBox.setText("Database error - can't update sensorLog table!");
            msgBox.exec();

            qCritical("Cannot delete: %s (%s)",
                      db.lastError().text().toLatin1().data(),
                      qt_error_string().toLocal8Bit().data());

            qDebug() << query.executedQuery();

            return;

        }
}

void MainWindow::on_pushButton_2_clicked()
{
   getLocations();
}

void MainWindow::on_sofa1CheckBox_toggled(bool checked)
{
   if (firstTime) return;

   QString stat = "Occupied";
   if (checked)
   {
      stat = "Free";
   }


   updateSensorLog(15, !checked, stat);
}

void MainWindow::on_fridgeCheckBox_toggled(bool checked)
{
    if (firstTime) return;

    QString stat = "On";
    if (checked)
    {
       stat = "Off";
    }

    updateSensorLog(50, !checked, stat);
}



void MainWindow::on_pushButton_clicked()
{

    updateSensorLog(59, 1, "On");

    Phonon::MediaObject *music = createPlayer(Phonon::MusicCategory,
                                                  Phonon::MediaSource("doorbell-1.wav"));
    music->play();

    updateSensorLog(59, 0, "Off");
}

void MainWindow::on_sofa2CheckBox_toggled(bool checked)
{
    if (firstTime) return;

    QString stat = "Occupied";
    if (checked)
    {
       stat = "Free";
    }

    updateSensorLog(16, !checked, stat);
}

void MainWindow::on_sofa3CheckBox_toggled(bool checked)
{
    if (firstTime) return;

    QString stat = "Occupied";
    if (checked)
    {
       stat = "Free";
    }

    updateSensorLog(17, !checked, stat);
}



void MainWindow::on_trayCheckBox_toggled(bool checked)
{
    if (firstTime) return;

    QString stat = "Empty";
    if (checked)
    {
       stat = "Full";
    }

    updateSensorLog(500, checked, stat);
}

void MainWindow::on_cupCheckBox_clicked(bool checked)
{
    if (firstTime) return;

    QString stat = "Empty";
    if (checked)
    {
       stat = "Full";
    }

    updateSensorLog(60, checked, stat);

}


void MainWindow::on_robotLocationComboBox_currentIndexChanged(QString locn)
{
     if (firstTime) return;

      QString loc;
      loc = locn.section("::",1,1);
      QString qry;
      QString actR;
      actR.setNum(activeRobot);
      qry = "UPDATE Robot SET locationId = ";
      qry += loc + " where robotId = " +  actR;

      qDebug()<<qry;

      QSqlQuery query(qry);

      query.exec();


}

void MainWindow::on_userLocationComboBox_currentIndexChanged(QString locn)
{
    QString loc;
    loc = locn.section("::",1,1);
    QString qry;
    qry = "UPDATE Users SET locationId = " +  loc + " where userId = " + activeUser;

    QSqlQuery query(qry);

    query.exec();
}


void MainWindow::on_sofa4CheckBox_clicked(bool checked)
{
    if (firstTime) return;

    QString stat = "Occupied";
    if (checked)
    {
       stat = "Free";
    }

    updateSensorLog(18, !checked, stat);
}

void MainWindow::on_sofa5CheckBox_clicked(bool checked)
{
    if (firstTime) return;

    QString stat = "Occupied";
    if (checked)
    {
       stat = "Free";
    }

    updateSensorLog(19, !checked, stat);
}

void MainWindow::on_TVCheckBox_clicked(bool checked)
{
    if (firstTime) return;

    QString stat = "Off";
    if (checked)
    {
       stat = "On";
    }

    updateSensorLog(49, checked, stat);
}

void MainWindow::on_trayStatusCheckBox_clicked(bool checked)
{
    if (firstTime) return;

    QString stat = "Lowered";
    if (checked)
    {
       stat = "Raised";
    }

    updateSensorLog(501, checked, stat);
}


void MainWindow::on_kettleCheckBox_clicked(bool checked)
{
    if (firstTime) return;

    QString stat = "On";
    if (!checked)
    {
       stat = "Off";
    }

    updateSensorLog(51, checked, stat);
}

void MainWindow::on_microCheckBox_clicked(bool checked)
{
    if (firstTime) return;

    QString stat = "On";
    if (!checked)
    {
       stat = "Off";
    }

    updateSensorLog(54, checked, stat);
}

void MainWindow::on_toasterCheckBox_clicked(bool checked)
{
    if (firstTime) return;

    QString stat = "On";
    if (!checked)
    {
       stat = "Off";
    }

    updateSensorLog(56, checked, stat);
}
