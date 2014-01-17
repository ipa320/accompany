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
//#include <phonon/audiooutput.h>
//#include <phonon/mediaobject.h>

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
    QString host, user, pw;
    bool ok;

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

     ui->runningAtLabel->setText(lv);

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

    db = QSqlDatabase::addDatabase("QMYSQL");

    db.setHostName(host);
    db.setDatabaseName("Accompany");
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

    }

    // hard codede as this is for ZUYD only

    experimentLocation = 2;
    activeRobot = 3;



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

     locQuery += " AND ((L1.locationId > 599 AND L1.locationId < 700) or L1.locationId = 999) ORDER BY L1.locationId";

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

   locQuery = "SELECT locationId FROM Robot where robotId = 3 LIMIT 1";

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


        locQuery += " AND ((L1.locationId > 599 AND L1.locationId < 700) OR L1.locationId = 999) ORDER BY L1.locationId";

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

   locQuery = "SELECT value from Sensors where sensorId = 301 LIMIT 1";

   query = locQuery;

   query.exec();

   while(query.next())
   {
      if (query.value(0).toInt() == 0)
      {
         ui->sofa1CheckBox->setChecked(false);
      }
      else
      {
         ui->sofa1CheckBox->setChecked(true);
      }
   }

   locQuery = "SELECT value from Sensors where sensorId = 304 LIMIT 1";

   query = locQuery;

   query.exec();

   while(query.next())
   {
      if (query.value(0).toInt() == 0)
      {
         ui->sofa2CheckBox->setChecked(false);
      }
      else
      {
         ui->sofa2CheckBox->setChecked(true);
      }
   }

   locQuery = "SELECT value from Sensors where sensorId = 305 LIMIT 1";
   query.clear();
   query = locQuery;

   query.exec();

   while(query.next())
   {
      if (query.value(0).toInt() == 0)
      {
         ui->sofa3CheckBox->setChecked(false);
      }
      else
      {
         ui->sofa3CheckBox->setChecked(true);
      }
   }

   locQuery = "SELECT value from Sensors where sensorId = 500 LIMIT 1";

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



   locQuery = "SELECT value from Sensors where sensorId = 307 LIMIT 1";

   query = locQuery;

   query.exec();

   while(query.next())
   {
      ui->cupCheckBox->setChecked(query.value(0).toBool());
   }

   locQuery = "SELECT value from Sensors where sensorId = 303 LIMIT 1";

   query = locQuery;

   query.exec();

   while(query.next())
   {
      if (query.value(0).toInt() == 0)
      {
         ui->fridgeCheckBox->setChecked(true);
      }
      else
      {
         ui->fridgeCheckBox->setChecked(false);
      }
   }

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

   QString stat = "Free";
   if (checked)
   {
      stat = "Occupied";
   }

   updateSensorLog(301, checked, stat);
}

void MainWindow::on_fridgeCheckBox_toggled(bool checked)
{
    if (firstTime) return;

    QString stat = "Open";
    if (checked)
    {
       stat = "Closed";
    }

    updateSensorLog(303, !checked, stat);
}



void MainWindow::on_pushButton_clicked()
{

    updateSensorLog(302, 1, "On");

    //Phonon::MediaObject *music = createPlayer(Phonon::MusicCategory,
    //                                              Phonon::MediaSource("doorbell-1.wav"));
    //music->play();

    updateSensorLog(302, 0, "Off");
}

void MainWindow::on_sofa2CheckBox_toggled(bool checked)
{
    if (firstTime) return;

    QString stat = "Free";
    if (checked)
    {
       stat = "Occupied";
    }

    updateSensorLog(304, checked, stat);
}

void MainWindow::on_sofa3CheckBox_toggled(bool checked)
{
    if (firstTime) return;

    QString stat = "Free";
    if (checked)
    {
       stat = "Occupied";
    }

    updateSensorLog(305, checked, stat);
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

    updateSensorLog(307, checked, stat);

}


void MainWindow::on_robotLocationComboBox_currentIndexChanged(QString locn)
{

      QString loc;
      loc = locn.section("::",1,1);
      QString qry;
      qry = "UPDATE Robot SET locationId = " +  loc + " where robotId = 3";

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


