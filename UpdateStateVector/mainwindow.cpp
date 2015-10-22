#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QInputDialog>
#include <QDebug>
#include <QMessageBox>


QSqlDatabase db;
bool dbOpen;
QString experimentLocation;
QTimer *timer;
int updates;


MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    closeDownRequest = false;

    timer = new QTimer();
    connect(timer, SIGNAL(timeout()), this, SLOT(createStateVector()));

    updates = 0;
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

    ui->locnlabel->setText(lv + ":" + user + ":" + host + ":" + dBase);


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
    ui->stopPushButton->setEnabled(false);





}

void MainWindow::createStateVector()
{
//    qDebug()<<"called";
   QSqlQuery query;

   query.exec(" select lastUpdate, sensorId, status from Sensors where sensorId < 61 order by sensorId");

   QDateTime start(QDate(2010,1,1),QTime(0,0,1));
   QString lastSensorId = "-1";
   QDateTime currentUpdate;

 //  qDebug()<<start;

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
       seq = "update SensorVector set sensor" + sensorId + " =  '" + sensorStatus + "'";

   //    qDebug()<<seq;

       updateQuery.exec(seq);

   }

   query.clear();

   query.exec("select lastUpdate from SensorVector");       // this will be the previous time it was updated
   query.next();
   currentUpdate = query.value(0).toDateTime();

   if (currentUpdate == start)    // same as latest
   {
       qDebug()<<"No update necessary - no change occurred";
       return;
   }

//   then update with max lastUpdate
//   qDebug()<<start;

   QSqlQuery updateQuery;

   QString seq;
   seq = "update SensorVector set lastUpdate = '" + start.toString("yyyy-MM-dd hh:mm:ss") + "', lastSensorId = '" + lastSensorId + "'";

  // qDebug()<<seq;

   updateQuery.exec(seq);

   seq = "insert into SensorVectorHistory select * from SensorVector";

   updateQuery.exec(seq);

   ui->lcdNumber->display(updates++);

}

void MainWindow::on_startPushButton_clicked()
{
    ui->stopPushButton->setEnabled(true);
    ui->startPushButton->setEnabled(false);

    qDebug()<<ui->spinBox->value() * 1000;

    timer->start(ui->spinBox->value() * 1000);

}

void MainWindow::on_stopPushButton_clicked()
{
    ui->startPushButton->setEnabled(true);
    ui->stopPushButton->setEnabled(false);

    timer->stop();
}
