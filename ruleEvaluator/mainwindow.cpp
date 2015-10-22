#include "mainwindow.h"
#include "ui_mainwindow.h"

QTimer  *timer;




QSqlDatabase db;
bool dbOpen;

QString defaultRobot = "::0::Care-O-Bot 3.2";
QString defaultUser  = "1";
QString defaultRobotStr = "0";

int experimentLocation;   // 1 = UH, 2=HUYT 3=Madopa
QString expLocation;



MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    closeDownRequest = false;

    ui->tableWidget->clear();
    ui->tableWidget->setColumnCount(2);

    ui->tableWidget->verticalHeader()->setVisible(false);

    QStringList labs;
    labs << "Name" << "Probability";
    ui->tableWidget->setHorizontalHeaderLabels(labs);

    ui->startPushButton->setEnabled(true);
    ui->stopPushButton->setEnabled(false);

    timer = new QTimer();

    connect(timer, SIGNAL(timeout()), this, SLOT(doClassification()));

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

    return true;
}

void MainWindow::doClassification()
{

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

     ui->tableWidget->setRowCount(query.size());

    int row = 0;

    while (query.next())
    {

        createCase(query.value(0).toString());    // create a 1 row vector of current on disk

        float conf = classify(query.value(0).toString());  // this does the classification

        if (conf == -1) return;

        QString sConf;
        sConf.setNum(conf,'g',1);

        ui->tableWidget->setItem(row, 0, new QTableWidgetItem(query.value(0).toString()));
        ui->tableWidget->item(row, 0)->setBackgroundColor(Qt::white);

        QTableWidgetItem *item = new QTableWidgetItem();
        item->setTextAlignment(Qt::AlignCenter);
        item->setText(sConf);
        ui->tableWidget->setItem(row, 1, item);

        bool context = false;
        if (conf > 0.5)
        {
            ui->tableWidget->item(row, 1)->setBackgroundColor(Qt::green);
            context = true;
        }
        else
        {
            ui->tableWidget->item(row, 1)->setBackgroundColor(Qt::red);
        }

        if (!updateContextSensor(query.value(0).toString(),context)) return;

        ui->tableWidget->resizeColumnsToContents();

        row++;


    }

}


void MainWindow::createCase(QString classification)
{

   qDebug()<<classification;

   QSqlQuery query;

   QString tempTable = "tempClass" + classification.simplified().remove(" ");

   query.exec("DROP TEMPORARY TABLE IF EXISTS " + tempTable);

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
                 + "', classification = '?'";

    //   qDebug()<<seq;
    //   qDebug()<<"...update temp final";
       updateQuery.exec(seq);

    // create the class file for C5.0


    QString fname;
    fname = "/home/joe/git/accompany/ShowME/data/" + classification + ".cases";

    QFile file(fname);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text))
    {
        qDebug()<<"Error opening class file";
        return;
    }

    QTextStream out(&file);

    seq = "select * from " + tempTable;
    updateQuery.clear();
    updateQuery.exec(seq);


    QSqlRecord rec = updateQuery.record();
    int cols =  rec.count();

    while(updateQuery.next())
    {
        for (int i=0;i<cols-1;i++)
        {
            out << updateQuery.value(i).toString() <<",";
        }

        out << updateQuery.value(cols-1).toString()<<"\n";
     }

    file.close();

}



float MainWindow::classify(QString item)
{

   FILE *pf;

   // Execute a process listing

   QString command = "~/C5-example/c5-example -f /home/joe/git/accompany/ShowME/data/" + item + " > /home/joe/git/accompany/ShowME/data/" + item + ".res";

   qDebug()<<command;

   // Setup our pipe for reading and execute our command.

   pf = popen(command.toUtf8().constData(),"r");

   if(!pf)
   {
     qDebug()<<"Could not open pipe for output.\n";
     return -1;
   }


   if (pclose(pf) != 0)
   {
       qDebug()<<" Error: Failed to close command stream";
       return -1;
   }

   QString fname = "/home/joe/git/accompany/ShowME/data/" + item + ".res";
   QFile PFfile(fname);

   if (!PFfile.open(QIODevice::ReadOnly | QIODevice::Text))
   {
       qDebug()<<"No results file";
       return -1;
   }


   QTextStream inPF(&PFfile);
   QString PFline = inPF.readLine();

   float Confidence;

   while (!PFline.isNull())
   {
      QString s = PFline.section("]",0,0).section("[",1,1);
      Confidence = PFline.section("[",2,2).section("]",0,0).toFloat();

      if (s == item)
      {
         return Confidence;
      }
      else
      {
         Confidence = 0;
      }

      PFline = inPF.readLine();
   }

   return Confidence;
}

bool MainWindow::updateContextSensor(QString context, bool outcome)
{
    QSqlQuery query;

    QString seqQuery;
    seqQuery = "SELECT * FROM Sensors ";
    seqQuery+= "WHERE name = '" + context + "' AND sensorId >899 and sensorId < 1000 LIMIT 1 ";

    qDebug()<<seqQuery;

    query.prepare(seqQuery);

    if (!query.exec())
    {
        qDebug() << query.lastQuery();

        QMessageBox msgBox;
        msgBox.setIcon(QMessageBox::Critical);

        msgBox.setText("Can't select from sensors table?");
        msgBox.exec();

        return 0;
    }

    if (query.next())    // Sensor already exists therefore update it
    {
        QString sOutcome = "false";
        if (outcome) sOutcome = "true";
        QString bOutcome;
        bOutcome.setNum(outcome);

        // first check if the value has changed - this ensures that we dont update unnecsariyl and modify the update time

        if (query.value(1).toBool() == outcome) return 1;     // nothing to do


        seqQuery  = "INSERT INTO SensorLog (timestamp,sensorId,room,channel,value,status) VALUES (NOW(), ";
        seqQuery  += query.value(0).toString() + ",'',''," + bOutcome + ",'" + sOutcome + "')";

        qDebug()<< seqQuery;

        query.clear();

        query.prepare(seqQuery);

        if (!query.exec())
        {
           qDebug() << query.lastQuery();

           QMessageBox msgBox;
           msgBox.setIcon(QMessageBox::Critical);

           msgBox.setText("Can't insert into SensorLog table");
           msgBox.exec();

           return 0;
        }

    }
    else
    {
       query.clear();

       query.prepare("SELECT MAX(sensorId) FROM Sensors where sensorId >899 and sensorId < 1000 ");

       if (!query.exec())
       {
          qDebug() << query.lastQuery();

          QMessageBox msgBox;
          msgBox.setIcon(QMessageBox::Critical);

          msgBox.setText("Can't select from Sensors table");
          msgBox.exec();

          return 0;
       }

       int sId;

       while(query.next())
       {

          if (query.value(0).toInt() == 0)
          {
             sId = 900;
          }
          else
          {
             sId = query.value(0).toInt() + 1;
          }

         qDebug() << sId;

         if (outcome)
         {
            seqQuery = "INSERT INTO Sensors VALUES (:sensorId, '1', '1', :name, '5', 'Predicate', 'N/A', '6',NOW(),NOW(),0,'true','true',NULL,NULL,NULL,NULL)";
         }
         else
         {
            seqQuery = "INSERT INTO Sensors VALUES (:sensorId, '0', '0', :name, '5', 'Predicate', 'N/A', '6',NOW(),NOW(),0,'false','false',NULL,NULL,NULL,NULL)";
         }

         query.prepare(seqQuery);

         query.bindValue(":sensorId",sId);
         query.bindValue(":name",context);


          if (!query.exec())
          {

            qDebug() << query.lastQuery();

            QMessageBox msgBox;
            msgBox.setIcon(QMessageBox::Critical);

            msgBox.setText("Can't add to Sensors table - duplicate?");
            msgBox.exec();

            return 0;
          }
      }
   }

    return 1;

}


void MainWindow::on_stopPushButton_clicked()
{
    ui->startPushButton->setEnabled(true);
    ui->stopPushButton->setEnabled(false);

    timer->stop();
}

void MainWindow::on_startPushButton_clicked()
{
    ui->startPushButton->setEnabled(false);
    ui->stopPushButton->setEnabled(true);


    timer->start(1000);
}
