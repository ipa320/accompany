#include "mainwindow.h"
#include "ui_mainwindow.h"

QSqlDatabase db;
bool dbOpen;

int experimentLocation;
int defaultUserId;
int minOffset;
int hourOffset;

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
    ui->locnLabel->setText(lv + ":" + user + ":" + host + ":" + dBase) ;
    // fill the fields

    QSqlQuery query("SELECT ExperimentalLocationId, SessionUser, sessionTimeOffsetMin, SessionTimeOffsetHours FROM SessionControl WHERE SessionId = 1 LIMIT 1");
    query.exec();
    if (query.next())
    {
       experimentLocation = query.value(0).toInt();
       defaultUserId = query.value(1).toInt();
       minOffset = query.value(2).toInt();
       hourOffset = query.value(3).toInt();
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

    query.clear();

    query.prepare("SELECT location FROM ExperimentalLocation");

    query.exec();

    while (query.next())
    {
        ui->locationComboBox->addItem(query.value(0).toString());
    }

    ui->locationComboBox->setCurrentIndex(experimentLocation - 1);

    query.clear();

    query.prepare("SELECT nickname, userId FROM Users");

    query.exec();

    QString currentUser;

    while (query.next())
    {
        ui->userComboBox->addItem(query.value(0).toString() + "::" + query.value(1).toString() + "::");

        if (query.value(1).toInt() == defaultUserId)
        {
            currentUser = query.value(0).toString() + "::" + query.value(1).toString() + "::";
        }

    }

    ui->userComboBox->setCurrentIndex(ui->userComboBox->findText(currentUser));

    ui->minuteSpinBox->setValue(minOffset);

    ui->hourSpinBox->setValue(hourOffset);

    connect(&timer, SIGNAL(timeout()), this, SLOT(updateTime()));

    timer.start(100);


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

void MainWindow::updateTime()
{

  //  qDebug()<<"timer";
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

    query.exec("CALL spGetSchedulerTime()");

    while(query.next())
    {
      ui->schedulerTime->setText(query.value(0).toString());
    }

}

void MainWindow::on_locationComboBox_currentIndexChanged(int index)
{

    QString user;
    QString userInd;
    userInd.setNum(index +1);

    user = "UPDATE SessionControl SET ExperimentalLocationId = " + userInd  + " where sessionId = 1";

    QSqlQuery query(user);
    query.exec();

}

void MainWindow::on_userComboBox_currentIndexChanged(QString user)
{

    QString userInd = user.section("::",1,1);


    user = "UPDATE SessionControl SET SessionUser = " + userInd  + " where sessionId = 1";

    QSqlQuery query(user);
    query.exec();
}

void MainWindow::on_minuteSpinBox_valueChanged(int min)
{
    QString userInd;
    userInd.setNum(min);


    QString user = "UPDATE SessionControl SET sessionTimeOffsetMin = " + userInd  + " where sessionId = 1";

    QSqlQuery query(user);
    query.exec();

}

void MainWindow::on_hourSpinBox_valueChanged(int hours)
{
    QString userInd;
    userInd.setNum(hours);


    QString user = "UPDATE SessionControl SET sessionTimeOffsetHours = " + userInd  + " where sessionId = 1";

    QSqlQuery query(user);
    query.exec();
}
