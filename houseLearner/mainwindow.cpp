#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <QInputDialog>
#include <QDebug>
#include <QMessageBox>


QSqlDatabase db;
bool dbOpen;
QString experimentLocation;

QDateTime latestStateDate;

QSqlQuery query;

bool processed;
int numRowsRead;
int numRowsProcessed;
int numRowsInserted;

QDateTime currentDate;
QString   currentSensorId;
QString   currentSensorValue;
QString   currentTrainingNumber;

QDateTime prevDate;
QString   prevSensorId;
QString   prevSensorValue;
QString   prevTrainingNumber;

QString behTable;


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

    QFile file;

    if (lv=="BATCH" || lv == "BATCHNORM")
    {
      file.setFileName("/home/joe/git/accompany/UHCore/Core/config.py");
    }
    else
    {
      file.setFileName("../UHCore/Core/config.py");
    }


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
    qDebug()<<lv << "," << user << "," << host << "," << dBase;

    if (lv != "BATCH" && lv!= "BATCHNORM")
    {
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

    }


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


    QSqlQuery expQuery("SELECT ExperimentalLocationId  FROM SessionControl WHERE SessionId = 1 LIMIT 1");

    if (expQuery.next())
    {
       experimentLocation = expQuery.value(0).toString();
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


    QString seqQuery;

    // get the date of the latest row in sensorStateHistory

    seqQuery = "select max(lastUpdate) from SensorStateHistory";
    query = seqQuery;

    if (!query.exec())
    {
        QMessageBox msgBox;
        msgBox.setIcon(QMessageBox::Warning);

        msgBox.setText("Cannot select from SensorStateHistory table!");
        msgBox.exec();
        return;
    }

    while (query.next())
    {
        latestStateDate = query.value(0).toDateTime();
    }

    ui->inflatePushButton->setEnabled(false);

//    QDateTime Test = QDateTime(QDate(2014,3,19),QTime(13,0,0));
    QDateTime Test1 = QDateTime(QDate(2030,01,01),QTime(0,0,0));


    ui->dateFrom->setDateTime(latestStateDate);
 //   ui->dateTo->setDateTime(latestStateDate);

 //     ui->dateFrom->setDateTime(Test);
      ui->dateTo->setDateTime(Test1);

      behTable = "BehaviourLog";

      if (lv=="BATCH")
      {
          behTable = "BehaviourLog";
          fillSensorStateTable();
          closeDownRequest = true;

      }

      if (lv=="BATCHNORM")
      {
          behTable = "NormBehaviourLog";
          on_inflatePushButton_clicked();  // create sensor states for every 1 second
          fillSensorStateTable();         // update the state table from the behaviourlog
          closeDownRequest = true;

      }

}


void MainWindow::fillSensorStateTable()
{
    if (!checkDates()) return;

    QString seqQuery;

    // get the latest row from the existing sensorStateHistory into a temporary table

    seqQuery  = "DROP TEMPORARY TABLE IF EXISTS tempState";

 //   qDebug() << seqQuery;

    query.clear();

    if (!query.exec(seqQuery))
    {
        QMessageBox msgBox;
        msgBox.setIcon(QMessageBox::Warning);

        msgBox.setText("Cannot drop temporary table tempState!");
        msgBox.exec();
        return;
    }

    seqQuery  = "select max(lastUpdate) from SensorStateHistory";

 //   qDebug() << seqQuery;

    query.clear();

    if (!query.exec(seqQuery))
    {
        QMessageBox msgBox;
        msgBox.setIcon(QMessageBox::Warning);

        msgBox.setText("Cannot select from sensorStateHistory!");
        msgBox.exec();
        return;
    }

    // insert the lateset row from sensorStateHistory to temp table

    QDateTime lastDate;

    while (query.next())
    {
        lastDate = query.value(0).toDateTime();

    }


    qDebug()<<"Latest date on sensorStateHistory is: " << lastDate.toString("yyyy-MM-dd hh:mm:ss");

    seqQuery  = "CREATE TEMPORARY TABLE tempState ( select * from SensorStateHistory where lastUpdate =  '";
    seqQuery += lastDate.toString("yyyy-MM-dd hh:mm:ss") + "')";


 //   qDebug() << seqQuery;

    query.clear();

    if (!query.exec(seqQuery))
    {
        QMessageBox msgBox;
        msgBox.setIcon(QMessageBox::Warning);

        msgBox.setText("Cannot create temporary table tempState!");
        msgBox.exec();

        qDebug()<<query.lastError();
        qDebug()<<query.executedQuery();


        return;
    }


    // get the current values from behaviourLog


    if (lv == "BATCH" || "BATCHNORM")
    {
        seqQuery  = "select timestamp,sensorId,status,trainingNumber from " + behTable + " where sensorID < 61 ORDER BY trainingNumber, timestamp";
    }
    else
    {
        seqQuery  = "select timestamp,sensorId,status,trainingNumber from " + behTable + " where sensorID < 61 and timestamp between '";
        seqQuery += ui->dateFrom->dateTime().toString("yyyy-MM-dd hh:mm:ss");
        seqQuery += "' and '";
        seqQuery += ui->dateTo->dateTime().toString("yyyy-MM-dd hh:mm:ss") + "' ORDER BY timestamp";
    }
    qDebug() << seqQuery;


    query.clear();

    if (!query.exec(seqQuery))
    {
        QMessageBox msgBox;
        msgBox.setIcon(QMessageBox::Warning);


        msgBox.setText("Cannot select from SensorLog table!");
        msgBox.exec();
        return;

    }



    int numRowsRead = query.numRowsAffected();
    int numRowsProcessed = 0;
    int numRowsInserted = 0;



    while (query.next())

    {

        if (numRowsProcessed%50 == 0)
        {
            qDebug()<<"Processed " << numRowsProcessed << " rows...";

        }



         qDebug()<<query.value(0).toDateTime().toString("yyyy-MM-dd hh:mm:ss");
        qDebug()<< query.value(1).toString();
        qDebug()<< query.value(2).toString();
        qDebug()<< query.value(3).toString();

        qDebug()<< lastDate.toString("yyyy-MM-dd hh:mm:ss");

        // update the temp table and insert to db

        if (query.value(0).toDateTime() == lastDate)  // dates the same, update row with sensor values
        {

            qDebug()<<"Same - updating temp table";

             if (!updateTempTable())
             {
                 QMessageBox msgBox;
                 msgBox.setIcon(QMessageBox::Warning);
                 msgBox.setText("Problem updating temp table!");
                 msgBox.exec();
                 return;
             }

            numRowsProcessed++;
        }

        if (query.value(0).toDateTime() > lastDate) // new event, insert a new stateHistory row
        {

            qDebug() << "Greater";

            if (numRowsProcessed > 0)   // insert the current row on temp to stateHistory
            {
       //         qDebug()<< "insert current temp to stateHistory";

                seqQuery  = "insert INTO SensorStateHistory select * from tempState";

      //         qDebug() << seqQuery;

                QSqlQuery insertQuery;

                insertQuery.clear();

                if (!insertQuery.exec(seqQuery))
                {
                    QMessageBox msgBox;
                    msgBox.setIcon(QMessageBox::Warning);

                    msgBox.setText("Cannot insert into sensorStateHistory!");
                    msgBox.exec();
                    msgBox.setText(insertQuery.lastError().text());
                    msgBox.exec();
                    qDebug()<<insertQuery.lastError();
                    qDebug()<<insertQuery.executedQuery();
                    return;
                }

                numRowsInserted++;

            }

            // now update it

            if (!updateTempTable())
            {
                QMessageBox msgBox;
                msgBox.setIcon(QMessageBox::Warning);
                msgBox.setText("Problem updating temp table!");
                msgBox.exec();
                return;
            }


            numRowsProcessed++;

//            QDateTime latestDate   = query.value(0).toDateTime();

//            int days = lastDate.daysTo ( latestDate ) ;

//            QTime when = QTime ( 0, 0, 0, 0 ) ;

//            when = when.addSecs ( lastDate.addDays(days).secsTo( latestDate ) ) ;

     //       qDebug()  << latestDate.toString( "M/d/yyyy h:mm:ss AP" )
     //             << " - "
     //             << lastDate.toString( "M/d/yyyy h:mm:ss AP" )
     //             << " = "
     //             << days << when.toString( ".HH:mm:ss" ) ;

            lastDate = query.value(0).toDateTime();
        }

    }

    // final row

    if (numRowsProcessed > 0)   // insert the current row on temp to stateHistory
    {
  //      qDebug()<< "insert final temp to stateHistory";

  //      seqQuery  = "insert INTO SensorStateHistory select * from tempState";

  //      qDebug() << seqQuery;

        QSqlQuery insertQuery;

        insertQuery.clear();

        if (!insertQuery.exec(seqQuery))
        {
            QMessageBox msgBox;
            msgBox.setIcon(QMessageBox::Warning);

            msgBox.setText("Cannot insert into sensorStateHistory!");
            msgBox.exec();
            msgBox.setText(insertQuery.lastError().text());
            msgBox.exec();
            qDebug()<<insertQuery.lastError();
            qDebug()<<insertQuery.executedQuery();
            return;
        }

        numRowsInserted++;
    }


    qDebug()<<"Rows Read: " << numRowsRead << "Rows Procesed: " << numRowsProcessed << "Rows inserted: " << numRowsInserted;


}

void MainWindow::clearSensorStateTable()
{

  //  if (!checkDates()) return;

    // clear the current sensorStateHistory

    QString seqQuery;
     ;

    // clear everything except the starting row

    seqQuery = "DELETE FROM SensorStateHistory where lastUpdate != '2012-01-01 00:00:01'";

    query.clear();

    if (!query.exec(seqQuery))
    {
        QMessageBox msgBox;
        msgBox.setIcon(QMessageBox::Warning);

        msgBox.setText("Cannot delete from existing SensorStateHistory table!");
        msgBox.exec();
        return;
    }


    // get the date of the latest row in sensorStateHistory

    seqQuery = "select max(lastUpdate) from SensorStateHistory";
    query = seqQuery;

    if (!query.exec())
    {
        QMessageBox msgBox;
        msgBox.setIcon(QMessageBox::Warning);

        msgBox.setText("Cannot select from SensorStateHistory table!");
        msgBox.exec();
        return;
    }

    while (query.next())
    {
        latestStateDate = query.value(0).toDateTime();
    }


}

void MainWindow::on_clearPushButton_clicked()
{
    clearSensorStateTable();

}

void MainWindow::on_fillPushButton_clicked()
{
    fillSensorStateTable();
}

bool MainWindow::checkDates()
{


    if (ui->dateFrom->dateTime() < latestStateDate )
    {

        QMessageBox msgBox;
        msgBox.setIcon(QMessageBox::Warning);

        msgBox.setText("*From* date is to early!");
        msgBox.exec();
        return false;
    }


    if (ui->dateTo->dateTime() < ui->dateFrom->dateTime())
    {

        QMessageBox msgBox;
        msgBox.setIcon(QMessageBox::Warning);

        msgBox.setText("*From* date must be before *to* date!");
        msgBox.exec();
        return false;
    }

    return true;
}

bool MainWindow::updateTempTable()
{

   // get the location - this will be the location at the level below 'robot house' e.g. rooms

    QSqlQuery locnQuery;

    QString qry;

    qry = "select";
    qry+= "       CASE  WHEN L1.where = 0 and L2.where = 0 and L3.where = 0 THEN L1.name";
    qry+= "             WHEN L3.where = 0 and L2.where = 0 THEN L2.name";
    qry+= "             ELSE L3.name END,";
    qry+= "       CASE  WHEN L1.where = 0 and L2.where = 0 and L3.where = 0 THEN L1.locationId";
    qry+= "             WHEN L3.where = 0 and L2.where = 0 THEN L2.locationId";
    qry+= "             ELSE L3.locationId END";
    qry+= "       FROM Locations L1, Locations L2, Locations L3";
    qry+= "       WHERE L2.locationId = L1.where";
    qry+= "         AND L3.locationId = L2.where";
    qry+= "         AND L1.locationId = (";
    qry+= "select L.locationId from Sensors S, Locations L where S.sensorId = ";
    qry+= query.value(1).toString() + " and L.locationId = S.locationId LIMIT 1)";

 //   qDebug()<<qry;

    if (!locnQuery.exec(qry))
    {
        QMessageBox msgBox;
        msgBox.setIcon(QMessageBox::Warning);

        msgBox.setText("Cannot select from location or sensor tables!");
        msgBox.exec();
        msgBox.setText(locnQuery.lastError().text());
        msgBox.exec();
        qDebug()<<locnQuery.lastError();
        qDebug()<<locnQuery.executedQuery();
        return false;
    }

    if (!locnQuery.next())
    {
        QMessageBox msgBox;
        msgBox.setIcon(QMessageBox::Warning);

        msgBox.setText("Cannot select row from location query!");
        msgBox.exec();
        msgBox.setText(locnQuery.lastError().text());
        msgBox.exec();
        qDebug()<<locnQuery.lastError();
        qDebug()<<locnQuery.executedQuery();
        return false;

    }

    qDebug()<<"**************************";
    QString seqQuery;
    seqQuery  = "UPDATE tempState SET lastUpdate = '" + query.value(0).toDateTime().toString("yyyy-MM-dd hh:mm:ss");
    seqQuery += "', trainingNumber = '" +  query.value(3).toString();
    seqQuery += "', sensor" + query.value(1).toString();
    seqQuery +=  " = '" + query.value(2).toString() + "', location = '" + locnQuery.value(0).toString() + "(" + locnQuery.value(1).toString() + ")'";

    qDebug() << seqQuery;

    QSqlQuery updateQuery;

    updateQuery.clear();

    if (!updateQuery.exec(seqQuery))
    {
        QMessageBox msgBox;
        msgBox.setIcon(QMessageBox::Warning);

        msgBox.setText("Cannot update tempState table!");
        msgBox.exec();
        msgBox.setText(updateQuery.lastError().text());
        msgBox.exec();
        qDebug()<<updateQuery.lastError();
        qDebug()<<updateQuery.executedQuery();
        return false;
    }

    return true;
}


void MainWindow::on_inflatePushButton_clicked()
{


     QSqlQuery delQuery;
     delQuery.exec("delete from NormBehaviourLog");


    QString seqQuery;

    if (lv == "BATCHNORM" || lv == "BATCH")
    {
        seqQuery  = "select timestamp,sensorId,status,trainingNumber from BehaviourLog where sensorID < 61 ORDER BY trainingNumber, timestamp";
    }
    else
    {
       seqQuery  = "select timestamp,sensorId,status,trainingNumber from BehaviourLog where sensorID < 61 and timestamp between '";
       seqQuery += ui->dateFrom->dateTime().toString("yyyy-MM-dd hh:mm:ss");
       seqQuery += "' and '";
       seqQuery += ui->dateTo->dateTime().toString("yyyy-MM-dd hh:mm:ss") + "' ORDER BY timestamp";
    }
    qDebug() << seqQuery;


    query.clear();

    if (!query.exec(seqQuery))
    {
        qDebug() << seqQuery;

        QMessageBox msgBox;
        msgBox.setIcon(QMessageBox::Warning);


        msgBox.setText("Cannot select from BehaviourLog table!");
        msgBox.exec();
        return;

    }

    numRowsRead = query.numRowsAffected();
    numRowsProcessed = 0;
    numRowsInserted = 0;

    bool first = true;


    processed = false;

    currentDate = QDateTime(QDate(2000,01,01),QTime(0,0,1));

    readRow();

    while (!processed)
    {

      if (first)
      {
          first = false;
          insertNewRow(currentDate,currentSensorId,currentSensorValue,currentTrainingNumber);

       }
       else
       {

            if ((currentDate > prevDate.addSecs(1))  &&  (currentTrainingNumber == prevTrainingNumber))   // write n copies of current row 1 sec apart
            {
                int n = prevDate.time().msecsTo(currentDate.time()) / 1000;

                n--;

                QDateTime insDate = prevDate;

                for (int i=0;i<n;i++)
                {
                    insDate = insDate.addSecs(1);

                    insertNewRow(insDate,prevSensorId,prevSensorValue,prevTrainingNumber);

                }

                insertNewRow(currentDate,currentSensorId,currentSensorValue, currentTrainingNumber);


            }
            else   // same dates for two events
            {
                insertNewRow(currentDate,currentSensorId,currentSensorValue, currentTrainingNumber);
            }


         }

         readRow();

    }

    qDebug()<< "Rows read   : " << numRowsRead;
    qDebug()<< "Rows processed   : " << numRowsProcessed;
    qDebug()<< "Rows written: " << numRowsInserted;
}


void MainWindow::readRow()
{

    prevDate           =  currentDate;
    prevSensorId       =  currentSensorId;
    prevSensorValue    =  currentSensorValue;
    prevTrainingNumber =  currentTrainingNumber;

    if (!query.next())
    {
        processed = true;
    }
    else
    {

 //     qDebug()<<query.value(0).toDateTime().toString() << " "<< query.value(1).toString() << " " << query.value(2).toString() << " " << query.value(3).toString();



      currentDate            =  query.value(0).toDateTime();
      currentSensorId        =  query.value(1).toString();
      currentSensorValue     =  query.value(2).toString();
      currentTrainingNumber  =  query.value(3).toString();

      numRowsProcessed++;
    }
}


void MainWindow::insertNewRow(QDateTime newDate, QString sensorId, QString sensorValue, QString trainNo)
{
    QString ins;
    QSqlQuery insQuery;

    ins =  "INSERT INTO NormBehaviourLog VALUES (";
    ins += "\"" + newDate.toString("yyyy-MM-dd hh:mm:ss")  + "\"";
    ins += "," + sensorId;
    ins += ",'" + sensorValue + "'";
    ins += "," + trainNo + ")";

    insQuery.prepare(ins);

    insQuery.exec(ins);

    numRowsInserted++;

    if (numRowsInserted % 50 == 0)
    {
        qDebug()<<"Inserted " << numRowsInserted << " rows...";

    }

}

void MainWindow::on_checkBox_toggled(bool checked)
{
    ui->inflatePushButton->setEnabled(checked);
    if (checked)
    {
       behTable = "NormBehaviourLog";
    }
    else
    {
       behTable = "BehaviourLog";
    }
}

void MainWindow::on_checkBox_2_clicked(bool checked)
{
    if (checked) lv="BATCHNORM";
}
