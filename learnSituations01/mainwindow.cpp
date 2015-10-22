#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QInputDialog>
#include <QDebug>
#include <QMessageBox>


QSqlDatabase db;
bool dbOpen;
QString experimentLocation;



QDateTime ruleFrom;
QDateTime ruleTo;
QDateTime dateFrom;
QDateTime dateTo;
QString className;
QString trainingNoStr;

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
    qDebug()<<lv;

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


    QSqlQuery query;

    query.clear();

    QString qry;

    qry = "SELECT DISTINCT name FROM showMeList";

    query.prepare(qry);

    query.exec();

    ui->activityComboBox->clear();

    ui->activityComboBox->setEnabled(true);

    while(query.next())
    {
       ui->activityComboBox->addItem(query.value(0).toString());
    }

    ui->activityComboBox->setEnabled(true);

}


void MainWindow::on_behLogPushButton_clicked()
{

    QSqlQuery query;
    QSqlQuery insertQuery;
    QString seqQuery;


    // clear the existing behaviourLog

    query.exec("DELETE FROM BehaviourLog");

//    seqQuery = "SELECT appStartTime, appEndTime from showMeList where name = '" + ui->activityComboBox->currentText() +"'";

//    if (!query.exec(seqQuery))
//    {
//       QMessageBox msgBox;
//       msgBox.setIcon(QMessageBox::Warning);
//       msgBox.setText("Cannot select from ShowMeList table!");
//       msgBox.exec();
//       return;
//     }

    seqQuery = "SELECT name, startTime, endTime from showMeList where name = '" + ui->activityComboBox->currentText() +"' order by id";

    qDebug() << seqQuery;

    if (!query.exec(seqQuery))
     {
        QMessageBox msgBox;
        msgBox.setIcon(QMessageBox::Warning);
        msgBox.setText("Cannot select from ShowMeList table!");
        msgBox.exec();
        return;
     }


 int trainingNumber = 0;

 while (query.next())
 {

        // insert two dummy rows for the app start and app end dates - this is because these mark where
        // the training program started and the trainingLog will match on realtive times between these dates/times


        dateFrom  = query.value(1).toDateTime();
        dateTo    = query.value(2).toDateTime();
        className = query.value(0).toString();
        trainingNoStr.setNum(trainingNumber);


/*
        dateFrom = query.value(0).toDateTime();
        dateTo = query.value(1).toDateTime();

        QString trainingNoStr;
        trainingNoStr.setNum(trainingNumber);

        seqQuery  =  "insert into BehaviourLog values ";
        seqQuery += "('" + dateFrom.toString("yyyy-MM-dd hh:mm:ss") + "','45', 'Off','" + trainingNoStr +"')";

        qDebug() << seqQuery;

        insertQuery.clear();

        insertQuery.exec(seqQuery);


        seqQuery  =  "insert into BehaviourLog values ";
        seqQuery += "('" + dateTo.toString("yyyy-MM-dd hh:mm:ss") + "','45', 'Off','" + trainingNoStr +"')";

        qDebug() << seqQuery;

        insertQuery.clear();

        insertQuery.exec(seqQuery);
*/
    // get the current values from sensorLog between app start and end times

        seqQuery  =  "insert into BehaviourLog ";
        seqQuery += "select timestamp,sensorId,status, " + trainingNoStr + ",'" + className + "' from SensorLog where sensorID < 61 and timestamp between '";
        seqQuery += dateFrom.toString("yyyy-MM-dd hh:mm:ss");
        seqQuery += "' and '";
        seqQuery += dateTo.toString("yyyy-MM-dd hh:mm:ss") + "' ORDER BY timestamp";

        qDebug() << seqQuery;

        insertQuery.clear();

       if (!insertQuery.exec(seqQuery))
       {
          QMessageBox msgBox;
          msgBox.setIcon(QMessageBox::Warning);


          msgBox.setText("Cannot insert/select from SensorLog table!");
          msgBox.exec();
          return;
        }

        trainingNumber++;
    }

}

void MainWindow::on_ruleSetPushButton_clicked()
{

    QSqlQuery query;
    QSqlQuery insertQuery;
    QString seqQuery;


    // clear the existing TeachingLog

    query.exec("DELETE FROM TeachingLog");

    seqQuery = "SELECT startTime, endTime, appStartTime, appEndTime from showMeList where name = '" + ui->activityComboBox->currentText() +"'";

    if (!query.exec(seqQuery))
    {
       QMessageBox msgBox;
       msgBox.setIcon(QMessageBox::Warning);


       msgBox.setText("Cannot select from ShowMeList table!");
       msgBox.exec();
       return;
     }

    int trainingNumber = 0;

    while (query.next())
    {

        QString trainingNoStr;
        trainingNoStr.setNum(trainingNumber);

        ruleFrom = query.value(0).toDateTime();
        ruleTo = query.value(1).toDateTime();
        dateFrom = query.value(2).toDateTime();
        dateTo = query.value(3).toDateTime();

        if (ruleFrom == dateFrom)
        {
            trainingLogInsert(ruleFrom, ui->activityComboBox->currentText(), trainingNoStr);
        }
        else
        {
            trainingLogInsert(dateFrom, "Undefined", trainingNoStr);
        }

        trainingLogInsert(ruleFrom, ui->activityComboBox->currentText(), trainingNoStr);
        trainingLogInsert(ruleTo, "Undefined", trainingNoStr);
        trainingLogInsert(dateTo, "Undefined", trainingNoStr);

        trainingNumber++;
    }


    // update relative time

    QSqlQuery updateQuery;

    updateQuery.exec("drop table if exists classTimings");

    updateQuery.clear();

    seqQuery  = "CREATE TABLE classTimings AS ";
    seqQuery += "select ";
    seqQuery += "     (TIMEDIFF(S.lastUpdate,@starttm)) as relativeTime,";
    seqQuery += "               @lasttm := S.lastUpdate as actualTime ";
    seqQuery += "from TeachingLog S, ( select @starttm := lastUpdate, @lasttm := lastUpdate from TeachingLog order by trainingNumber,lastUpdate limit 1) StartRow ";
    seqQuery += "   order by S.trainingNumber, S.lastUpdate";

    if (!updateQuery.exec(seqQuery))
    {
       QMessageBox msgBox;
       msgBox.setIcon(QMessageBox::Warning);


       msgBox.setText("Cannot create classTimings table!");
       msgBox.exec();
       return;
     }

    updateQuery.clear();

    seqQuery  =  "update TeachingLog S, classTimings R set S.relativeTime = R.relativeTime where S.lastUpdate = R.actualTime";

    if (!updateQuery.exec(seqQuery))
    {
       QMessageBox msgBox;
       msgBox.setIcon(QMessageBox::Warning);


       msgBox.setText("Cannot update classTimings table!");
       msgBox.exec();
       return;
     }

}

bool MainWindow::trainingLogInsert(QDateTime tDate, QString txt, QString trainingNoStr)
{

    QSqlQuery insertQuery;
    QString seqQuery;

    seqQuery  =  "insert into TeachingLog ";
    seqQuery += "values ('";
    seqQuery += tDate.toString("yyyy-MM-dd hh:mm:ss");
    seqQuery += "','" + txt + "', ";
    seqQuery += trainingNoStr + ",'00:00:00')";

    qDebug() << seqQuery;

    insertQuery.clear();

    if (!insertQuery.exec(seqQuery))
    {
       QMessageBox msgBox;
       msgBox.setIcon(QMessageBox::Warning);
       msgBox.setText("Cannot insert/select into TrainingLog table!");
       msgBox.exec();
       return false;
    }

    return true;
}


void MainWindow::on_ruleStatePushButton_clicked()
{

    // create an empty stateHistory table

    FILE *pf;

    // Execute a process

    QString command = "mysql -h localhost -u rhUser -pwaterloo TeachMeTest < scripts/createSensorStateHistory.sql";

    // Setup our pipe for reading and execute our command.

    pf = popen(command.toUtf8().constData(),"r");

    if(!pf)
    {
      qDebug()<<"Could not open pipe for output.\n";
      return;
    }

    if (pclose(pf) != 0)
    {
        qDebug()<<" Error: Failed to close command stream";
        return;
    }


    // now fill it with 'undefined'  rows - same number as classification rows in behaviour log

    QSqlQuery query;
    QSqlQuery updQuery;
    QSqlQuery insQuery;
    QString seqQuery;

    seqQuery = "SELECT COUNT(*) from BehaviourLog";

    qDebug() << seqQuery;

    if (!query.exec(seqQuery))
     {
        QMessageBox msgBox;
        msgBox.setIcon(QMessageBox::Warning);
        msgBox.setText("Cannot select from BehaviourLog table!");
        msgBox.exec();
        return;
     }

     query.next();

     command = "mysql -h localhost -u rhUser -pwaterloo TeachMeTest < scripts/insertSensorStateHistory.sql";

     for (int i=0;i<query.value(0).toInt();i++)
     {
         // Setup our pipe for reading and execute our command.

         pf = popen(command.toUtf8().constData(),"r");

         if(!pf)
         {
           qDebug()<<"Could not open pipe for output.\n";
           return;
         }

         if (pclose(pf) != 0)
         {
             qDebug()<<" Error: Failed to close command stream";
             return;
         }


     }

     ///////////
     // first get the current state from the sensors table


     // now fill with behaviourLog rows in state vector format

     // get a  row from the existing sensorStateHistory into a temporary table

     seqQuery  = "DROP TEMPORARY TABLE IF EXISTS tempState";

     if (!query.exec(seqQuery))
     {
         QMessageBox msgBox;
         msgBox.setIcon(QMessageBox::Warning);

         msgBox.setText("Cannot drop temporary table tempState!");
         msgBox.exec();
         return;
     }

     seqQuery  = "CREATE TEMPORARY TABLE tempState ( select * from SensorStateHistory limit 1)";

     qDebug() << seqQuery;

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

     seqQuery  = "select sensorId, status from Sensors where sensorId < 61 order by sensorId";

     if (!query.exec(seqQuery))
     {
         QMessageBox msgBox;
         msgBox.setIcon(QMessageBox::Warning);
         msgBox.setText("Cannot select from Sensors table!");
         msgBox.exec();
         qDebug()<<query.lastError();
         qDebug()<<query.executedQuery();
         return;

     }

     QString lastSensorId = "-1";


   //  qDebug()<<start;

     while (query.next())
     {

         QString sensorId           = query.value(0).toString();
         QString sensorStatus       = query.value(1).toString();

         seQuery = "update tempState set sensor" + sensorId + " =  '" + sensorStatus + "'";

         qDebug()<<seqQuery;

         if (updQuery.exec(seq))
         {
             QMessageBox msgBox;
             msgBox.setIcon(QMessageBox::Warning);
             msgBox.setText("Cannot update from Sensors table!");
             msgBox.exec();
             qDebug()<<query.lastError();
             qDebug()<<query.executedQuery();
             return;
         }

     }



  //   then update with max lastUpdate
  //   qDebug()<<start;

     QSqlQuery updateQuery;

     QString seq;
     seq = "update SensorVector set lastUpdate = '" + start.toString("yyyy-MM-dd hh:mm:ss") + "', lastSensorId = '" + lastSensorId + "'";

    // qDebug()<<seq;

     updateQuery.exec(seq);

     seq = "insert into SensorVectorHistory select * from tempState";




     ///////////




     // get the current values from behaviourLog

     seqQuery  = "select sensorId,status,class from BehaviourLog";

     qDebug() << seqQuery;

     if (!query.exec(seqQuery))
      {
         QMessageBox msgBox;
         msgBox.setIcon(QMessageBox::Warning);
         msgBox.setText("Cannot select from BehaviourLog table!");
         msgBox.exec();
         return;
      }

      while (query.next())
      {
          seqQuery = "UPDATE tempState set sensor" + query.value(0).toString() + " = '" + query.value(1).toString() + "', class = '" + query.value(2).toString() + "'";
          qDebug()<<seqQuery;

          if (!updQuery.exec(seqQuery))
          {
              QMessageBox msgBox;
              msgBox.setIcon(QMessageBox::Warning);
              msgBox.setText("Cannot update temp table!");
              msgBox.exec();
              return;
          }

          seqQuery = "INSERT INTO SensorStateHistory SELECT * FROM tempState";


          if (!insQuery.exec(seqQuery))
          {
              QMessageBox msgBox;
              msgBox.setIcon(QMessageBox::Warning);
              msgBox.setText("Cannot insert from temp table to sensorStateTable!");
              msgBox.exec();
              return;
          }


      }







//  Updating sensorHistory with senslog info...
/*
    command = "/home/joe/git/accompany/houseLearner/houseLearner -l BATCHNORM";

    pf = popen(command.toUtf8().constData(),"r");

    if(!pf)
    {
      qDebug()<<"Could not open pipe for output.\n";
      return;
    }

    if (pclose(pf) != 0)
    {
        qDebug()<<" Error: Failed to close command stream";
        return;
    }


//  Creating ruleStateHistory...

    command = "mysql -h localhost -u rhUser -pwaterloo TeachMeTest < /home/joe/Documents/ACCOMPANY/Ismael/houseLearner/scripts/ruleStateHistoryCreate.sql";

    pf = popen(command.toUtf8().constData(),"r");

    if(!pf)
    {
       qDebug()<<"Could not open pipe for output.\n";
       return;
    }

    if (pclose(pf) != 0)
    {
       qDebug()<<" Error: Failed to close command stream";
       return;
    }
*/
}

void MainWindow::on_classUpdatePushButton_clicked()
{
    FILE *pf;

    // Execute a process

    QString command = "/home/joe/git/accompany/classUpdate/classUpdate -l BATCH";

    // Setup our pipe for reading and execute our command.

    pf = popen(command.toUtf8().constData(),"r");

    if(!pf)
    {
      qDebug()<<"Could not open pipe for output.\n";
      return;
    }

    if (pclose(pf) != 0)
    {
        qDebug()<<" Error: Failed to close command stream";
        return;
    }

}

void MainWindow::on_namesPushButton_clicked()
{


    // appends the showMe name to a C5.0 template names file - thus listing all the house sensors
    // plus two possible classifications undefined and the showMe name (e.g. undefined and watchingTV)

    QSqlQuery query;
    QString seqQuery;

    seqQuery = "SELECT DISTINCT name from showMeList";

    qDebug() << seqQuery;

    if (!query.exec(seqQuery))
     {
        QMessageBox msgBox;
        msgBox.setIcon(QMessageBox::Warning);
        msgBox.setText("Cannot select from ShowMeList table!");
        msgBox.exec();
        return;
     }

     while (query.next())
     {
        qDebug()<<query.value(0);

        FILE *pf;

        // Execute a process

        QString command = "cp scripts/ruleNames.template scripts/" + query.value(0).toString() + ".names";

        qDebug()<<command;

        // Setup our pipe for reading and execute our command.

        pf = popen(command.toUtf8().constData(),"r");

        if(!pf)
        {
          qDebug()<<"Could not open pipe for output.\n";
          return;
        }

        if (pclose(pf) != 0)
        {
            qDebug()<<" Error: Failed to close command stream";
            return;
        }

        command = "echo '                                        " + query.value(0).toString() + "' >> scripts/" +  query.value(0).toString() + ".names";

        // Setup our pipe for reading and execute our command.

        pf = popen(command.toUtf8().constData(),"r");

        if(!pf)
        {
          qDebug()<<"Could not open pipe for output.\n";
          return;
        }

        if (pclose(pf) != 0)
        {
            qDebug()<<" Error: Failed to close command stream";
            return;
        }


     }
}
