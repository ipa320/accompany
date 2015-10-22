#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <QInputDialog>
#include <QDebug>
#include <QMessageBox>


QSqlDatabase db;
bool dbOpen;
QString experimentLocation;


QSqlQuery stateQuery;
QSqlQuery classQuery;

QTime classTime;
QString classString;
QString classTrainNum;

QString prevClassString;
QString prevTrainNum;

QTime stateTime;
QTime endTime;
QString stateTrainNum;

QString jobStatus;




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

    if (lv=="BATCH")
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

    if (lv != "BATCH")
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


    QString strg = "select * from ruleStateHistory order by trainingNumber, relativeTime";
    stateQuery = strg;

    if (!stateQuery.exec())
    {
        QMessageBox msgBox;
        msgBox.setIcon(QMessageBox::Warning);

        msgBox.setText("Cannot select from ruleStateHistory table!");
        msgBox.exec();
        return;
    }


    strg = "select * from TeachingLog order by trainingNumber, relativeTime";
    classQuery = strg;

    if (!classQuery.exec())
    {
        QMessageBox msgBox;
        msgBox.setIcon(QMessageBox::Warning);

        msgBox.setText("Cannot select from TeachingLog table!");
        msgBox.exec();
        return;
    }

    prevClassString = "Undefined";
    endTime.setHMS(23,59,59,999);
    prevTrainNum = "-1";

    readStateTable();
    readClassTable();

    jobStatus = "Running";

    if (lv == "BATCH")
    {
       on_pushButton_2_clicked();
       closeDownRequest = true;
    }

  }

void MainWindow::on_pushButton_clicked()
{



    if ((classTime == stateTime) && (classTrainNum == stateTrainNum))
    {
        if ((classTime == endTime) and (classTrainNum == "999"))
        {
            qDebug()<<"Job Done";
            jobStatus = "Completed";
            return;
        }

        qDebug()<<"State: " << stateTime << " Class: " << classTime << " class: " << classString << " classTrain: " << classTrainNum << "stateTrain: " << stateTrainNum  ;

        updateStateTable(classString);

        prevClassString = classString;
        prevTrainNum    = classTrainNum;

        readClassTable();
        readStateTable();
    }
    else
    {
        if ((classTime > stateTime) && (classTrainNum == stateTrainNum)  )
        {
            qDebug()<<"State: " << stateTime << " Class: " << classTime << " pclass: " << prevClassString << " pclassTrain: " << prevTrainNum << "pstateTrain: " << prevTrainNum  ;;

            updateStateTable(prevClassString);

            readStateTable();
        }
        else
        {
            prevClassString = classString;
            prevTrainNum    = classTrainNum;

            readClassTable();
        }
    }



}


void MainWindow::readStateTable()
{

    // get the date of the latest row in sensorStateHistory


    if (stateQuery.next())
    {
        stateTime     = stateQuery.value(63).toTime();
        stateTrainNum = stateQuery.value(1).toString();

        qDebug()<<"  State read: "<< stateTime <<" "<<stateTrainNum;
    }
    else
    {
        stateTime.setHMS(23,59,59,999);
        stateTrainNum = "999";

        qDebug()<<"End of table";
    }


}


void MainWindow::readClassTable()
{

    // get the date of the latest row in teachingLog


    if (classQuery.next())
    {
        classTime     = classQuery.value(3).toTime();
        classString   = classQuery.value(1).toString();
        classTrainNum = classQuery.value(2).toString();

        qDebug()<<"   Class read: "<<classTime<< " " << classString<< " " << classTrainNum;
    }
    else
    {
        classTime.setHMS(23,59,59,999);
        classTrainNum = "999";

        qDebug()<<"End of table";
    }


}

void MainWindow::updateStateTable(QString classValue)
{

    // update the class

    QString qry;
    qry = "update ruleStateHistory set class = '" + classValue + "' where relativeTime = '" + stateTime.toString() +"'";

//    qDebug()<<qry;

    QSqlQuery updateStateQuery = qry;


    if (!updateStateQuery.exec())
    {
        QMessageBox msgBox;
        msgBox.setIcon(QMessageBox::Warning);

        msgBox.setText("Cannot update ruleStateHistory table!");
        msgBox.exec();
        return;

    }




}


void MainWindow::on_pushButton_2_clicked()
{
    while (jobStatus != "Completed")
    {
        on_pushButton_clicked();
    }
}
