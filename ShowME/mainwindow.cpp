#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "ui_options.h"


QTimer  *timer;
int t = 0;


double timing = 1;

bool blinked1 = true;
bool blinked2 = true;
bool blinked3 = true;

QSqlDatabase db;
bool dbOpen;

QString defaultRobot = "::0::Care-O-Bot 3.2";
QString defaultUser  = "1";
QString defaultRobotStr = "0";

int experimentLocation;   // 1 = UH, 2=HUYT 3=Madopa
QString expLocation;

bool box1inProgress = false;
bool box2inProgress = false;
bool box3inProgress = false;

QString currentLevel1Task;
QString currentLevel2Task;
QString currentLevel3Task;

QDateTime currentLevel1Task_startTime;
QDateTime currentLevel2Task_startTime;
QDateTime currentLevel3Task_startTime;

QDateTime appStartTime;
QDateTime appEndTime;

bool createNegativeState = false;

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



bool MainWindow::openDatabase()
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
        return false;
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

    if (param != "BATCH")
    {
      user = QInputDialog::getText ( this, "Accompany DB", "User:",QLineEdit::Normal,
                                     user, &ok);
      if (!ok)
      {
       closeDownRequest = true;
       return false;
      }

      pw = QInputDialog::getText ( this, "Accompany DB", "Password:", QLineEdit::Password,
                                                                      pw, &ok);
      if (!ok)
      {
        closeDownRequest = true;
        return false;
      }


      host = QInputDialog::getText ( this, "Accompany DB", "Host:",QLineEdit::Normal,
                                     host, &ok);
       if (!ok)
      {
        closeDownRequest = true;
        return false;
       };

       dBase = QInputDialog::getText ( this, "Accompany DB", "Database:",QLineEdit::Normal,
                                     dBase, &ok);
       if (!ok)
       {
         closeDownRequest = true;
         return false;
       };
    }

    QString dbUser = "Database: " + user + ":" + host + ":" + dBase;



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


        return false;
    }

    qDebug() << "Database Opened";


    QSqlQuery query("SELECT ExperimentalLocationId, SessionUser FROM SessionControl WHERE SessionId = 1 LIMIT 1");

    if (query.next())
    {
       experimentLocation = query.value(0).toInt();
       expLocation.setNum(experimentLocation);
       defaultUser = query.value(1).toString();

       QSqlQuery locn("select location from ExperimentalLocation where  id = '" + expLocation + "'  LIMIT 1");
       locn.next();
       dbUser += " :: Location: " + locn.value(0).toString();

       QSqlQuery user("select nickname from Users where userId = '" + defaultUser +  "'  LIMIT 1");
       user.next();
       dbUser += " :: User: " + user.value(0).toString();



    }
    else
    {
        QMessageBox msgBox;
        msgBox.setIcon(QMessageBox::Critical);

        msgBox.setText("Can find session control table!");
        msgBox.exec();
        closeDownRequest = true;
        return false;
    }

    ui->dbLabel->setText(dbUser);


    // create the dummy sensorVector row in the DB

    QString command = "mysql -h localhost -u rhUser -pwaterloo TeachMeTest < /home/joe/git/accompany/ShowME/scripts/createSensorVector.sql";

    FILE *pf;

    pf = popen(command.toUtf8().constData(),"r");

    if(!pf)
    {
      qDebug()<<"Could not open pipe for output.\n";
      return false;
    }

    if (pclose(pf) != 0)
    {
       qDebug()<<" Error: Failed to close command stream" << command;
       return false;
    }

    // insert a row into sensorVector

    command = "mysql -h localhost -u rhUser -pwaterloo TeachMeTest < /home/joe/git/accompany/ShowME/scripts/insertSensorVector.sql";

    pf = popen(command.toUtf8().constData(),"r");

    if(!pf)
    {
      qDebug()<<"Could not open pipe for output.\n";
      return false;
    }

    if (pclose(pf) != 0)
    {
       qDebug()<<" Error: Failed to close command stream "<<command;
       return false;
    }

    timer = new QTimer();


     connect(timer, SIGNAL(timeout()), this, SLOT(blink()));

     timer->start(1000*timing);

     ui->showMeGroupBox_2->hide();
     ui->showMeGroupBox_3->hide();

     appStartTime = QDateTime::currentDateTime();
     qDebug()<<appStartTime.toString("yyyy-MM-dd hh:mm:ss");

     initialiseGUI();


    return true;

}


void MainWindow::initialiseGUI()
{

    if (!box1inProgress)
    {
        QSqlQuery query;

        query.clear();

        QString qry;

        qry = "SELECT DISTINCT name FROM showMeList";

        query.prepare(qry);

        query.exec();

        ui->showMeComboBox->clear();

        ui->showMeComboBox->setEnabled(false);

        while(query.next())
        {
           ui->showMeComboBox->addItem(query.value(0).toString());
        }


        ui->showMeComboBox->setCurrentIndex(-1);
        ui->showMeComboBox->clearEditText();


        ui->showMeStopPushButton->hide();
        ui->showMeForgetButton->hide();
        ui->showMePushButton->hide();
        ui->inProgressLabel1->setText("");


        ui->showMeComboBox->setEnabled(true);

    }

    if (!box2inProgress)
    {

        QSqlQuery query;

        query.clear();

        QString qry;

        qry = "SELECT DISTINCT name FROM showMeList";

        query.prepare(qry);

        query.exec();

        ui->showMeComboBox_2->clear();

        ui->showMeComboBox_2->setEnabled(false);

        while(query.next())
        {
           ui->showMeComboBox_2->addItem(query.value(0).toString());
        }


        ui->showMeComboBox_2->clearEditText();
        ui->showMeComboBox_2->setCurrentIndex(-1);

        ui->showMeStopPushButton_2->hide();
        ui->showMeForgetButton_2->hide();
        ui->showMePushButton_2->hide();
        ui->inProgressLabel2->setText("");



        ui->showMeComboBox_2->setEnabled(true);

    }

    if (!box3inProgress)
    {

        QSqlQuery query;

        query.clear();

        QString qry;

        qry = "SELECT DISTINCT name FROM showMeList";

        query.prepare(qry);

        query.exec();

        ui->showMeComboBox_3->clear();

        ui->showMeComboBox_3->setEnabled(false);

        while(query.next())
        {
           ui->showMeComboBox_3->addItem(query.value(0).toString());
        }


        ui->showMeComboBox_3->clearEditText();
        ui->showMeComboBox_3->setCurrentIndex(-1);

        ui->showMeStopPushButton_3->hide();
        ui->showMeForgetButton_3->hide();
        ui->showMePushButton_3->hide();
        ui->inProgressLabel3->setText("");


        ui->showMeComboBox_3->setEnabled(true);

    }

    if (!box1inProgress && !box2inProgress && !box3inProgress)
    {
        ui->showMeGroupBox_3->hide();
    }

    if (!box1inProgress && !box2inProgress)
    {
        ui->showMeGroupBox_2->hide();
    }

}


void MainWindow::on_showMePushButton_clicked()
{

      ui->showMeStopPushButton->show();

      ui->showMeForgetButton->hide();
      ui->showMePushButton->hide();

      box1inProgress = true;

      currentLevel1Task_startTime = QDateTime::currentDateTime();

      if (!box2inProgress)
      {
        ui->showMeGroupBox_2->show();
        ui->showMeComboBox_2->setEnabled(true);

        ui->showMeComboBox_2->clearEditText();

        ui->showMeStopPushButton_2->hide();
        ui->showMeForgetButton_2->hide();
        ui->showMePushButton_2->hide();

      }


}



void MainWindow::on_showMeComboBox_currentIndexChanged(const QString &arg1)
{

    ui->showMePushButton->show();
    ui->showMeForgetButton->hide();
    if (checkTaskExists(arg1))
    {
       ui->showMeForgetButton->show();
    }
    ui->showMeComboBox->setEnabled(false);

    currentLevel1Task = arg1.simplified().remove(" ");
}


void MainWindow::on_showMeForgetButton_clicked()
{

    if (!deleteActivity(currentLevel1Task))
    {
        return;
    }

    box1inProgress = false;
    initialiseGUI();

}

bool MainWindow::deleteActivity(QString activity)
{
    QSqlQuery query;

    query.prepare("DELETE FROM showMeList WHERE name = :name");

    query.bindValue(":name",activity);

    if (!query.exec())
    {


        QMessageBox msgBox;
        msgBox.setIcon(QMessageBox::Critical);

        msgBox.setText("Database error - can't delete from showMeList table!");
        msgBox.exec();

        qCritical("Cannot delete: %s (%s)",
                  db.lastError().text().toLatin1().data(),
                  qt_error_string().toLocal8Bit().data());
        return false;

    }

    query.prepare("DELETE FROM Sensors WHERE name = :name and sensorId > 899 and sensorId < 1000");

    query.bindValue(":name",activity);

    if (!query.exec())
    {


        QMessageBox msgBox;
        msgBox.setIcon(QMessageBox::Critical);

        msgBox.setText("Database error - can't delete from Sensors table!");
        msgBox.exec();

        qCritical("Cannot delete: %s (%s)",
                  db.lastError().text().toLatin1().data(),
                  qt_error_string().toLocal8Bit().data());
        return false;

    }

    query.prepare("DELETE FROM SensorVectorHistory where classification = :name");

    query.bindValue(":name",activity);

    if (!query.exec())
    {

        QMessageBox msgBox;
        msgBox.setIcon(QMessageBox::Critical);

        msgBox.setText("Database error - can't delete from SensorVectorHistory table!");
        msgBox.exec();

        qCritical("Cannot delete: %s (%s)",
                  db.lastError().text().toLatin1().data(),
                  qt_error_string().toLocal8Bit().data());
        return false;

    }

    return true;


}

void MainWindow::on_showMeStopPushButton_clicked()
{

 //   QDialog *timing = new QDialog() ;

 //   Ui_timing timingUi;
 //   timingUi.setupUi(timing);

 //   timing->exec();


    QSqlQuery query;


    qDebug()<<currentLevel1Task_startTime.toString("yyyy-MM-dd hh:mm:ss");
    qDebug()<<QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss");

    query.prepare("INSERT INTO showMeList (name,startTime,endTime,appStartTime) value (:name, :startTime, :endTime, :appStart)");

    query.bindValue(":name",currentLevel1Task);
    query.bindValue(":startTime",currentLevel1Task_startTime.toString("yyyy-MM-dd hh:mm:ss"));
    query.bindValue(":endTime",QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss"));
    query.bindValue(":appStart",appStartTime.toString("yyyy-MM-dd hh:mm:ss"));


    if (!query.exec())
    {
        QMessageBox msgBox;
        msgBox.setIcon(QMessageBox::Critical);

        msgBox.setText("Database error - can't insert into showMeList table!");
        msgBox.exec();

        qCritical("Cannot insert: %s (%s)",
                  db.lastError().text().toLatin1().data(),
                  qt_error_string().toLocal8Bit().data());
        return;

    }

    box1inProgress = false;
    initialiseGUI();

}



bool MainWindow::checkTaskExists(QString task)
{
    QSqlQuery query;

    query.prepare("SELECT name FROM showMeList WHERE name = :name limit 1");

    query.bindValue(":name",task);

    if (!query.exec())
    {
        QMessageBox msgBox;
        msgBox.setIcon(QMessageBox::Critical);

        msgBox.setText("Database error - can't select from showMeList table!");
        msgBox.exec();

        qCritical("Cannot select: %s (%s)",
                  db.lastError().text().toLatin1().data(),
                  qt_error_string().toLocal8Bit().data());
        return false;

    }

    if (query.next())
    {
        return true;
    }

    return false;
}



void MainWindow::on_showMeComboBox_2_currentIndexChanged(const QString &arg1)
{
    ui->showMePushButton_2->show();
    ui->showMeForgetButton_2->hide();
    if (checkTaskExists(arg1))
    {
       ui->showMeForgetButton_2->show();
    }
    ui->showMeComboBox_2->setEnabled(false);

    currentLevel2Task = arg1.simplified().remove(" ");
}

void MainWindow::on_showMePushButton_2_clicked()
{
    ui->showMeStopPushButton_2->show();

    ui->showMeForgetButton_2->hide();
    ui->showMePushButton_2->hide();

    currentLevel2Task_startTime = QDateTime::currentDateTime();

    box2inProgress = true;

    if (!box3inProgress)
    {
      ui->showMeGroupBox_3->show();
      ui->showMeComboBox_3->setEnabled(true);

      ui->showMeComboBox_3->clearEditText();

      ui->showMeStopPushButton_3->hide();
      ui->showMeForgetButton_3->hide();
      ui->showMePushButton_3->hide();


    }

}

void MainWindow::blink()
{
    qDebug()<<"Timer " << ++t ;


    if (box1inProgress || box2inProgress || box3inProgress )
    {}
    else
    {
        if (createNegativeState)
        {
            createStateVector("undefined");    // creates an undefined vector if no learning in progress
        }
        else
        {
            return;
        }
    }


    if (box1inProgress)
    {
       createStateVector(currentLevel1Task);
    }

    qDebug()<<"...end";

    if (box2inProgress)
    {
       createStateVector(currentLevel2Task);
    }

    if (box3inProgress)
    {
       createStateVector(currentLevel3Task);
    }

    if (blinked1 && box1inProgress)
    {
        ui->inProgressLabel1->setText("Learning in Progress...");
        blinked1 = false;
    }
    else
    {
        ui->inProgressLabel1->setText("");
        blinked1 = true;
    }

    if (blinked2 && box2inProgress)
    {
        ui->inProgressLabel2->setText("Learning in Progress...");
        blinked2 = false;
    }
    else
    {
        ui->inProgressLabel2->setText("");
        blinked2 = true;
    }

    if (blinked3 && box3inProgress)
    {
        ui->inProgressLabel3->setText("Learning in Progress...");
        blinked3 = false;
    }
    else
    {
        ui->inProgressLabel3->setText("");
        blinked3 = true;
    }

}










void MainWindow::on_showMeStopPushButton_2_clicked()
{
    QSqlQuery query;

    qDebug()<<currentLevel2Task_startTime.toString("yyyy-MM-dd hh:mm:ss");
    qDebug()<<QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss");

    query.prepare("INSERT INTO showMeList (name,startTime,endTime, appStartTime) value (:name, :startTime, :endTime, :appStart)");

    query.bindValue(":name",currentLevel2Task);
    query.bindValue(":startTime",currentLevel2Task_startTime.toString("yyyy-MM-dd hh:mm:ss"));
    query.bindValue(":endTime",QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss"));
    query.bindValue(":appStart",appStartTime.toString("yyyy-MM-dd hh:mm:ss"));

    if (!query.exec())
    {
        QMessageBox msgBox;
        msgBox.setIcon(QMessageBox::Critical);

        msgBox.setText("Database error - can't insert into showMeList table!");
        msgBox.exec();

        qCritical("Cannot insert: %s (%s)",
                  db.lastError().text().toLatin1().data(),
                  qt_error_string().toLocal8Bit().data());
        return;

    }

    box2inProgress = false;
    initialiseGUI();
}

void MainWindow::on_showMeForgetButton_2_clicked()
{
    if (!deleteActivity(currentLevel2Task))
    {
        return;
    }

    box1inProgress = false;
    initialiseGUI();
}

void MainWindow::on_showMeComboBox_3_currentIndexChanged(const QString &arg1)
{
    ui->showMePushButton_3->show();
    ui->showMeForgetButton_3->hide();
    if (checkTaskExists(arg1))
    {
       ui->showMeForgetButton_3->show();
    }
    ui->showMeComboBox_3->setEnabled(false);

    currentLevel3Task = arg1.simplified().remove(" ");
}

void MainWindow::on_showMePushButton_3_clicked()
{
    ui->showMeStopPushButton_3->show();

    ui->showMeForgetButton_3->hide();
    ui->showMePushButton_3->hide();

    currentLevel3Task_startTime = QDateTime::currentDateTime();

    box3inProgress = true;
}



void MainWindow::on_showMeStopPushButton_3_clicked()
{
    QSqlQuery query;

    qDebug()<<currentLevel3Task_startTime.toString("yyyy-MM-dd hh:mm:ss");
    qDebug()<<QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss");

    query.prepare("INSERT INTO showMeList (name,startTime,endTime,appStartTime) value (:name, :startTime, :endTime, :appStart)");

    query.bindValue(":name",currentLevel3Task);
    query.bindValue(":startTime",currentLevel3Task_startTime.toString("yyyy-MM-dd hh:mm:ss"));
    query.bindValue(":endTime",QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss"));
    query.bindValue(":appStart",appStartTime.toString("yyyy-MM-dd hh:mm:ss"));

    if (!query.exec())
    {
        QMessageBox msgBox;
        msgBox.setIcon(QMessageBox::Critical);

        msgBox.setText("Database error - can't insert into showMeList table!");
        msgBox.exec();

        qCritical("Cannot insert: %s (%s)",
                  db.lastError().text().toLatin1().data(),
                  qt_error_string().toLocal8Bit().data());
        return;

    }

    box3inProgress = false;
    initialiseGUI();
}

void MainWindow::on_showMeForgetButton_3_clicked()
{

    if (!deleteActivity(currentLevel3Task))
    {
        return;
    }

    box1inProgress = false;
    initialiseGUI();

}


void MainWindow::closeEvent (QCloseEvent *event)
{

    qDebug()<<"Closing";

    updateDataSets();
}

void MainWindow::updateDataSets()
{

    appEndTime = QDateTime::currentDateTime();
    qDebug()<<appEndTime.toString("yyyy-MM-dd hh:mm:ss");

    QSqlQuery query;

     query.prepare("UPDATE showMeList set appEndTime = :appEnd where appStartTime = :appStart");

    query.bindValue(":appStart",appStartTime.toString("yyyy-MM-dd hh:mm:ss"));
    query.bindValue(":appEnd",appEndTime.toString("yyyy-MM-dd hh:mm:ss"));

    if (!query.exec())
    {
        QMessageBox msgBox;
        msgBox.setIcon(QMessageBox::Critical);

        msgBox.setText("Database error - can't update showMeList table!");
        msgBox.exec();

        qCritical("Cannot insert: %s (%s)",
                  db.lastError().text().toLatin1().data(),
                  qt_error_string().toLocal8Bit().data());
        return;

    }

    // now update all C5.0 related bits and pieces

    // create a new names table for each classification in the showMeList


    // appends the showMe name to a C5.0 template names file - thus listing all the house sensors
    // plus two possible classifications undefined and the showMe name (e.g. undefined and watchingTV)


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

        // create a c5 names file withthe name of the activity

        QString compName = query.value(0).toString().simplified().remove(" ");

        QString command = "cp /home/joe/git/accompany/ShowME/scripts/ruleNames.template /home/joe/git/accompany/ShowME/data/" + compName + ".names";

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
            qDebug()<<" Error: Failed to close command stream" << command;
            return;
        }

        // append the attribute activity to the names file

        command = "echo '                                        " + compName + "' >> /home/joe/git/accompany/ShowME/data/" +  compName + ".names";

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
            qDebug()<<" Error: Failed to close command stream" << command;
            return;
        }


     }

    // create seperate training files for each classification activity  in sensorvectorhistory


     if (!query.exec(seqQuery))
      {
         QMessageBox msgBox;
         msgBox.setIcon(QMessageBox::Warning);
         msgBox.setText("Cannot select from ShowMeList table!");
         msgBox.exec();
         return;
      }

      FILE *pf;

      while (query.next())
      {
         qDebug()<<query.value(0);

         QString compName = query.value(0).toString().simplified().remove(" ");

         QString filename = "/home/joe/git/accompany/ShowME/data/" +compName + ".data";

         QFile file(filename);
         if (!file.open(QIODevice::WriteOnly | QIODevice::Text))
         {
             QMessageBox msgBox;
             msgBox.setIcon(QMessageBox::Warning);
             msgBox.setText("Problem opening data file for write!");
             msgBox.exec();
             return;
         }


         QTextStream out(&file);

         QSqlQuery insQuery;

         seqQuery  = "select * from SensorVectorHistory where ";
         seqQuery +="classification IN ('undefined','" + compName +"')";

         qDebug()<<seqQuery;


         insQuery.exec(seqQuery);

         QSqlRecord rec = insQuery.record();
         int cols =  rec.count();

         while(insQuery.next())
         {
             for (int i=0;i<cols-1;i++)
             {
                 out << insQuery.value(i).toString() <<",";
             }


             out << insQuery.value(cols-1).toString()<<"\n";

         }

/*
         seqQuery = "SELECT COUNT(*) FROM SensorVectorHistory WHERE classification = '" + query.value(0).toString() + "'";

    //     qDebug()<<seqQuery;

         insQuery.exec(seqQuery);

         insQuery.next();

         int rowCount = insQuery.value(0).toInt();


     //    qDebug()<<rowCount;

         seqQuery = "select * from SensorVector";

         insQuery.exec(seqQuery);

         insQuery.next();


         for (int k=0;k<rowCount;k++)
         {
             for (int i=0;i<cols-1;i++)
             {
                 out << insQuery.value(i).toString() <<",";
             }


             out << insQuery.value(cols-1).toString()<<"\n";
         }

*/

         file.close();


         // run C5.0 to create the trees

        // Execute a process listing



         QString command = "~/C50/c5.0 -b -w -f /home/joe/git/accompany/ShowME/data/" + compName;

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
           qDebug()<<" Error: Failed to close command stream" << command;
           return;
        }

      }







}

void MainWindow::createStateVector(QString classification)
{

   qDebug()<<classification;


   QSqlQuery query;

   QString tempTable = "temp" + classification.simplified().remove(" ");;

   query.exec("DROP TEMPORARY TABLE IF EXISTS " + tempTable);


//   query.exec("CREATE TEMPORARY TABLE " + tempTable + " (PRIMARY KEY (lastUpdate) USING BTREE) ( select * from SensorVector limit 1)");

  // the temprary table is created as below to ensure that the index is also created

   query.clear();

   query.exec("CREATE TEMPORARY TABLE " + tempTable + " LIKE SensorVector");

   query.clear();

   query.exec("INSERT INTO " + tempTable + " select * from SensorVector limit 1");


   query.clear();


   query.exec(" select lastUpdate, sensorId, status from Sensors where sensorId < 61 order by sensorId");

   QDateTime start(QDate(2010,1,1),QTime(0,0,1));
   QString lastSensorId = "-1";
   QDateTime currentUpdate;

//   qDebug()<<".sensor loop";

   while (query.next())
   {

       QDateTime sensorLastUpdate = query.value(0).toDateTime();
       QString sensorId           = query.value(1).toString();
       QString sensorStatus       = query.value(2).toString();


       if (sensorLastUpdate > start)  // this gets the last update
       {
           start = sensorLastUpdate;
           lastSensorId = sensorId;
       }

       QSqlQuery updateQuery;

       QString seq;
       seq = "update " + tempTable + " set sensor" + sensorId + " =  '" + sensorStatus + "'";

 //      qDebug()<<seq;
  //     qDebug()<<"...update temp loop";
       updateQuery.exec(seq);

   }

   query.clear();

   //// Sensor location

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
   qry+= lastSensorId + " and L.locationId = S.locationId LIMIT 1)";

 //  qDebug()<<qry;
//   qDebug()<<"..locn query";
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
       return;
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
       return;

   }


   QString location = locnQuery.value(0).toString();

   ////


   QSqlQuery updateQuery;

   QString seq;
   seq = "update " + tempTable
                   + " set lastUpdate = '" + start.toString("yyyy-MM-dd hh:mm:ss")
                   + "', sensorLocation = '" + location
                   + "', lastSensorId = '" + lastSensorId
                   + "', classification = '" + classification + "'";

//   qDebug()<<seq;
//   qDebug()<<"...update temp final";
   updateQuery.exec(seq);

   seq = "insert into SensorVectorHistory select * from " + tempTable;

  //    qDebug()<<seq;
//      qDebug()<<".... insert vector";

   updateQuery.exec(seq);

}



void MainWindow::on_timerPushButton_clicked()
{

   options opt(this); //create a object directly

  //    qDebug()<<timing;

   opt.setTiming(timing);
   opt.setNegativeExampleState(createNegativeState);

 //  qDebug()<<opt.getTiming();

   timer->stop();

   opt.exec(); //modal dialog

 //     qDebug()<<opt.getTiming();

   timing=opt.getTiming();

   createNegativeState = opt.getNegativeExampleState();

   timer->start(timing*1000);




}

void MainWindow::on_testPushButton_clicked()
{
    testShowMe test(this); //create a object directly

    timer->stop();

    updateDataSets();

    test.setTiming(timing);  // this sets the timing loop for the test dialog

    test.exec(); //modal dialog

    timer->start();  // this is the timing for the loop in this program
}

