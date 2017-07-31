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


#include <QSqlRelationalTableModel>
QSqlDatabase db;
bool dbOpen;

bool lastMin;
bool refresh;

QTimer timer;

MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent), ui(new Ui::MainWindow)
{
    ui->setupUi(this);





    dbOpen   = false;
    lastMin = false;
    refresh = false;

    ui->openDBButton->setEnabled(true);
    ui->robotButton->setEnabled(false);
    ui->peopleButton->setEnabled(false);
    ui->sensorCheckBox->setEnabled(false);
    ui->sensorsButton->setEnabled(false);
    ui->objectsButton->setEnabled(false);
    ui->locationsButton->setEnabled(false);
    ui->sensorLogButton->setEnabled(false);
    ui->emptyLogButton->setEnabled(false);
    ui->sensorDefaultsButton->setEnabled(false);
    ui->refreshButton->setEnabled(false);



}

MainWindow::~MainWindow()
{
    delete ui;
    QString connection;
    connection = db.connectionName();

    db.close();
    db = QSqlDatabase();
    db.removeDatabase(connection);

    qDebug() << "Database Closed";

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

void MainWindow::on_openDBButton_clicked()
{
   QString user = QInputDialog::getText ( this, "Accompany DB", "User:");
   QString pw = QInputDialog::getText ( this, "Accompany DB", "Password:", QLineEdit::Password);
   QString host = QInputDialog::getText ( this, "Accompany DB", "Host:");

   ui->locnLabel->setText(lv);

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

   ui->Userlabel->setText(user);
   ui->serverlabel->setText(host);

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
       return;
   }
   else {
       qDebug() << "Database Opened";
       ui->openDBButton->setEnabled(false);
       ui->robotButton->setEnabled(true);
       ui->peopleButton->setEnabled(true);
       ui->sensorCheckBox->setEnabled(true);
       ui->sensorsButton->setEnabled(true);
       ui->objectsButton->setEnabled(true);
       ui->locationsButton->setEnabled(true);
       ui->sensorLogButton->setEnabled(true);
       ui->emptyLogButton->setEnabled(true);
       ui->sensorDefaultsButton->setEnabled(true);
       ui->refreshButton->setEnabled(true);

   }




}





void MainWindow::on_locationsButton_clicked()
{
    if (timer.isActive() > 0) timer.stop();

    qDebug() << "Database queried";


   QSqlQueryModel *model = new QSqlQueryModel;

    model->setQuery("SELECT L1.locationId, L1.name, IF(STRCMP(L1.name,L2.name),L2.name,'')'In', IF(STRCMP(L2.name,L3.name),L3.name,'')'In', IF(STRCMP(L3.name,L4.name),L4.name,'')'In'\
                    FROM Locations L1,\
                         Locations L2,\
                         Locations L3,\
                         Locations L4\
                    WHERE L2.locationId = L1.where\
                      AND L3.locationId = L2.where\
                      AND L4.locationId = L3.where\
                    ORDER BY L2.name");

    ui->locationsView->setModel(model);
    ui->locationsView->resizeColumnsToContents();

    ui->locationsView->show();



}

void MainWindow::on_objectsButton_clicked()
{
    if (timer.isActive() > 0) timer.stop();

    QSqlQueryModel *model = new QSqlQueryModel;

    model->setQuery("SELECT O.objectId, O.name,  C.objectType, L.name'location', IF(O.graspable,'Yes','No')'Graspable', IF(O.moveable,'Yes','No')'Moveable'\
                    FROM Objects O,\
                         Locations L,\
                         ObjectCategory C\
                    where O.locationId       = L.locationId\
                      and O.objectCategoryId = C.objectCategoryId");

    ui->locationsView->setModel(model);
    ui->locationsView->resizeColumnsToContents();
    ui->locationsView->show();

}

void MainWindow::on_sensorsButton_clicked()
{
   if (timer.isActive()) timer.stop();
   if (refresh)
   {
    connect(&timer, SIGNAL(timeout()), this, SLOT(updateSensors()));
    timer.start(1000);
   }
   else
   {
       updateSensors();
   }
}


void MainWindow::updateSensors()
{
    QSqlQueryModel *model = new QSqlQueryModel;



//    model->setQuery("SELECT DATE_FORMAT(lastUpdate, '%W %M %Y %H:%i:%s'), DATE_FORMAT(NOW(), '%W %M %Y %H:%i:%s'), TIMESTAMPDIFF(SECOND,lastUpdate,NOW()) FROM Sensors");


    if (lastMin)
            model->setQuery("SELECT S.sensorId, S.name, L.name'Location',TIMESTAMPDIFF(SECOND,S.lastUpdate,NOW())'seconds',\
                            S.value,S.lastUpdate,T.sensorType,T.madeBy\
                            FROM Sensors S, Locations L, SensorType T\
                            WHERE S.locationId = L.locationId\
                              AND S.sensorTypeId = T.sensorTypeId\
                              AND TIMESTAMPDIFF(SECOND,S.lastUpdate,NOW()) < 60\
                            ORDER BY S.sensorId");
    else

        model->setQuery("SELECT S.sensorId, S.name, L.name'Location',TIMESTAMPDIFF(SECOND,S.lastUpdate,NOW())'seconds',\
                        S.value,S.lastUpdate,T.sensorType,T.madeBy\
                        FROM Sensors S, Locations L, SensorType T\
                        WHERE S.locationId = L.locationId\
                          AND S.sensorTypeId = T.sensorTypeId\
                        ORDER BY  S.sensorId");

    ui->locationsView->setModel(model);
    ui->locationsView->resizeColumnsToContents();
    ui->locationsView->show();
}



void MainWindow::on_peopleButton_clicked()
{
    if (timer.isActive() > 0) timer.stop();

    QSqlQueryModel *model = new QSqlQueryModel;

    model->setQuery("SELECT U.userId, U.nickname, L.name 'location', P.poseType'pose' \
                    FROM Users U,\
                         Locations L, \
                         Pose P\
                     where U.locationId = L.locationId\
                     and  U.poseId = P.poseId");

    ui->locationsView->setModel(model);
    ui->locationsView->resizeColumnsToContents();
    ui->locationsView->show();
}

void MainWindow::on_robotButton_clicked()
{

    if (timer.isActive() > 0) timer.stop();

    QSqlQueryModel *model = new QSqlQueryModel;

    model->setQuery("SELECT R.robotId, R.robotName, L.name'location',IF(R.robotStatus,'Running','Off')'status' \
                    FROM Robot R,\
                         Locations L\
                    where R.locationId = L.locationId");

    ui->locationsView->setModel(model);
    ui->locationsView->resizeColumnsToContents();
    ui->locationsView->show();
}

void MainWindow::on_sensorLogButton_clicked()
{
    if (timer.isActive() > 0) timer.stop();

    if (refresh)
    {
       connect(&timer, SIGNAL(timeout()), this, SLOT(updateSensorLog()));
       timer.start(1000);
    }
    else
    {
       updateSensorLog();
    }
}

void MainWindow::updateSensorLog()
{
    QSqlQueryModel *model = new QSqlQueryModel;

    if (lastMin)
    {
       model->setQuery("SELECT *,TIMESTAMPDIFF(SECOND,timestamp,NOW())'seconds'\
                    FROM SensorLog\
                    WHERE TIMESTAMPDIFF(SECOND,timestamp,NOW()) < 60\
                    ");
    }
    else
    {
        model->setQuery("SELECT *,TIMESTAMPDIFF(SECOND,timestamp,NOW())'seconds' FROM SensorLog");
    }

    ui->locationsView->setModel(model);
    ui->locationsView->resizeColumnsToContents();
    ui->locationsView->show();
}

void MainWindow::on_emptyLogButton_clicked()
{
     QMessageBox msgBox;
     msgBox.setIcon(QMessageBox::Warning);

     msgBox.setText("This will delete the contents of the sensorLog table!");
     msgBox.setInformativeText("Do you want to continue?");
     msgBox.setStandardButtons(QMessageBox::Yes| QMessageBox::No);
     msgBox.setDefaultButton(QMessageBox::No);
     int ret = msgBox.exec();

     switch (ret)
     {
     case QMessageBox::Yes:
        {
            QSqlQuery query;
            query.exec("DELETE FROM SensorLog");
            break;
        }
     case QMessageBox::No:
         // Don't delete was clicked
         break;
     default:
         // should never be reached
         break;
     }

}

void MainWindow::on_sensorDefaultsButton_clicked()
{
    QMessageBox msgBox;
    msgBox.setIcon(QMessageBox::Warning);

    msgBox.setText("This will set all sensors to their default value!");
    msgBox.setInformativeText("Do you want to continue?");
    msgBox.setStandardButtons(QMessageBox::Yes| QMessageBox::No);
    msgBox.setDefaultButton(QMessageBox::No);
    int ret = msgBox.exec();

    switch (ret)
    {
    case QMessageBox::Yes:
       {
           QSqlQuery query;
           query.exec("UPDATE Sensors SET value = 0, lastUpdate = NOW()");
           break;
       }
    case QMessageBox::No:
        // Don't delete was clicked
        break;
    default:
        // should never be reached
        break;
    }



}







void MainWindow::on_sensorCheckBox_stateChanged(int )
{
    if (lastMin)
        lastMin = false;
    else
        lastMin = true;
}

void MainWindow::on_refreshButton_stateChanged(int )
{
    if (refresh)
    {
        refresh = false;
        if (timer.isActive() > 0) timer.stop();
    }
    else
    {
        refresh = true;
    }
}
