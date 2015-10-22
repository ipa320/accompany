#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QMessageBox>
#include <QInputDialog>
#include <QtSql>
#include <QStringListModel>
#include <QStringList>


QSqlDatabase db;
bool dbOpen;



//QStringListModel *learnedModel;
    QStringList learnedList;

int okVisibleFlag;

QString defaultRobot = "::0::Care-O-Bot 3.2";
QString defaultUser  = "1";
QString defaultRobotStr = "0";

int globalReset;

int experimentLocation;   // 1 = UH, 2=HUYT 3=Madopa
QString expLocation;


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


    initialiseGUI();

    return true;

}


void MainWindow::initialiseGUI()
{

       ui->startLearningPushButton->hide();
       ui->stopLearningPushButton->hide();
       ui->learnedSoFarLabel->hide();


       ui->whatPushButton->hide();

       ui->whenPushButton->hide();
       ui->whatPushButton_2->hide();
       ui->whenPushButton_2->hide();

       ui->finishedB->hide();
       ui->finsihedB2->hide();

       ui->arrow1->hide();
       ui->arrow2->hide();



       ui->behNameComboBox->hide();
       ui->nickName->hide();

       ui->behNameLineEdit->hide();
       ui->cancelPushButton->hide();
       ui->alreadyKnowWidget->hide();
       ui->learnedSoFarListWidget->hide();

       ui->learningStartedLabel->hide();

       ui->teachMePushButton->setStyleSheet("QPushButton { background-color : green; color : white; }");
       ui->showMePushButton->setStyleSheet("QPushButton { background-color : green; color : white; }");

       ui->teachMePushButton->setEnabled(true);
       ui->showMePushButton->setEnabled(true);

       ui->teachMePushButton->show();
       ui->showMePushButton->show();

       QSqlQuery query;

       query.clear();

       QString qry;

       qry = "SELECT * FROM Sequences where scenario = 'User Generated' and name not like 'reset%' AND experimentalLocationId = " +  expLocation;

       query.prepare(qry);

       query.exec();

       ui->behNameComboBox->clear();

       while(query.next())
       {
          ui->behNameComboBox->addItem(query.value(0).toString());
       }

       ui->behNameComboBox->clearEditText();

       ui->behNameLineEdit->setText("Enter what you want the robot to do in this box");

       ui->addPushButton->hide();
       ui->deletePushButton->hide();
       ui->changePushButton->hide();

       //learnedModel = new QStringListModel();

       ui->rememberThisPushButton->hide();

       ui->okPushButton->hide();

       ui->allDonePushButton->hide();

       ui->learnedSoFarListWidget->clear();


       ui->resetSpinBox->setValue(60);

       ui->toldFinishedPushButton->setChecked(false);
       ui->whenTellMePushButton->setChecked(false);
       on_whenTellMePushButton_clicked(false);

       ui->houseFinishedPushButton->setChecked(false);
       ui->whenSomethingPushButton->setChecked(false);
       on_whenSomethingPushButton_clicked(false);

       ui->contextFinishedPushButton->setChecked(false);
       ui->doingPushButton->setChecked(false);
       on_doingPushButton_clicked(false);

       ui->locnFinishedPushButton->setChecked(false);
       ui->locationPushButton->setChecked(false);
       on_locationPushButton_clicked(false);

       ui->timeFinishedPushButton->setChecked(false);
       ui->timePushButton->setChecked(false);
       on_timePushButton_clicked(false);

       ui->happeningSpinBox->setEnabled(false);
       ui->hasHappendedSpinBox->setEnabled(false);
       ui->happeningSpinBox->setValue(0);
       ui->hasHappendedSpinBox->setValue(0);

       ui->alwaysCheckBox->setChecked(true);
       ui->resetSpinBox->setValue(0);


       ui->whenTellMePushButton->hide();
  //     ui->doingPushButton->hide();

}

void MainWindow::on_teachMePushButton_clicked()
{

    ui->showMePushButton->hide();
    ui->teachMePushButton->hide();
    ui->nickName->show();

 //   ui->teachMePushButton->setEnabled(false);
 //   ui->teachMePushButton->setStyleSheet("QPushButton { background-color : yellow; color : black; }");

    ui->behNameComboBox->show();
    ui->behNameComboBox->setEnabled(true);
    ui->behNameComboBox->setStyleSheet("QComboBox { background-color : white; color : black; }");
    ui->behNameComboBox->setFocus();

    ui->behNameLineEdit->show();
    ui->behNameLineEdit->setEnabled(true);
    ui->behNameLineEdit->setStyleSheet("QLineEdit { background-color : white; color : black; }");

    ui->cancelPushButton->show();

}

void MainWindow::on_cancelPushButton_clicked()
{

      initialiseGUI();
      //delete learnedModel;
}

void MainWindow::on_showMePushButton_clicked()
{

    // Set up the command line which starts the external program as QStringList.
        QStringList commandAndParameters;

        /* Fill in the following things:
         * - Name of program to execute.
         * - Every option needed by the program.
         * Attention: be aware of any strange options you pass to the program, e.g.
         * IP addresses. Quoting these options will usually help.
         */
        commandAndParameters                         <<"-b";

        /* Create a QProcess instance. It does not matter if it is created on the stack or
         * on the heap. - Ahem, I tested it on Linux only. :-)
         */
        QProcess myProcess;

        // Start the QProcess instance.
  //      myProcess.start("/home/joe/git/accompany/TeachMeShowMe/TeachMeShowMe", commandAndParameters);
        myProcess.start("/home/joe/git/accompany/ShowME/ShowME", commandAndParameters);
        /* O.k., everything is fine now, leave the Qt application. The external program
         * will continue running.
         */

        myProcess.waitForFinished(-1);
}

void MainWindow::on_startLearningPushButton_clicked()
{
     ui->behNameComboBox->setEnabled(false);
     ui->behNameLineEdit->setEnabled(false);

     ui->deletePushButton->hide();
     ui->changePushButton->hide();
     ui->addPushButton->hide();

     ui->startLearningPushButton->hide();
     ui->stopLearningPushButton->hide();

     ui->alreadyKnowWidget->setEnabled(true);
     ui->alreadyKnowWidget->show();

  //   ui->learningStartedLabel->setStyleSheet("QLabel { background-color : green; color : white; qproperty-alignment: AlignCenter;}");
  //   ui->learningStartedLabel->setText("Learning In Progress");
  //   ui->learningStartedLabel->show();

     ui->learnedSoFarLabel->show();
     ui->learnedSoFarListWidget->show();
   //  ui->learnedSoFarListView->show();
   //  ui->learnedSoFarListView->setStyleSheet("QListView { background-color : lightGrey; color : black; }");
   //  ui->learnedSoFarLabel->setStyleSheet("QLabel {  color : black; }");

 //    ui->stopLearningPushButton->show();

     ui->whatPushButton->show();
     ui->whenPushButton_2->show();

     ui->finsihedB2->show();


     ui->arrow1->show();
     ui->arrow2->show();

     for (int i=0;i<12;i++)
     {
       ui->alreadyKnowWidget->setTabEnabled(i,false);
     }

     ui->alreadyKnowWidget->setCurrentIndex(1);
     ui->alreadyKnowWidget->setCurrentIndex(0);   // do it twice to emit the changed signal

     ui->alreadyKnowWidget->setTabEnabled(0,true);
     ui->alreadyKnowWidget->setTabEnabled(1,true);
     ui->alreadyKnowWidget->setTabEnabled(2,true);
     ui->alreadyKnowWidget->setTabEnabled(3,true);
     ui->alreadyKnowWidget->setTabEnabled(4,true);

//     ui->alreadyKnowWidget->setCurrentIndex(1);

     ui->rememberThisPushButton->show();
     ui->locnFinishedPushButton->hide();
     ui->timeFinishedPushButton->hide();
     ui->houseFinishedPushButton->hide();
     ui->contextFinishedPushButton->hide();

     // fill the learned widget

     QSqlQuery query;

     query.prepare("SELECT * FROM ActionRules where name = '" + ui->behNameComboBox->currentText() + " AND experimentalLocationId = '" +  expLocation + "' order by ruleOrder");

     query.exec();

     while (query.next())
     {
         if (query.value(2) == "R" && (query.value(5).toString().left(1)!="7" && query.value(5).toString().left(1)!="8")) // rules, exclude resets
         {
                ui->learnedSoFarListWidget->addItem(query.value(5).toString());
         }

        if (query.value(2) == "A")
        {
           if (query.value(7).toString().left(5) == "speak")
           {
             ui->learnedSoFarListWidget->addItem("say \"" + query.value(7).toString().section(",",2,2) + "\"");
           }
           if (query.value(7).toString().left(8) == "sequence")
           {
               QSqlQuery query1;


               query1.prepare("SELECT * FROM Sequences where name = '" + query.value(7).toString().section(",",2,2) + "' AND experimentalLocationId = '" +  expLocation + "'");

               query1.exec();

               while (query1.next())
               {
        //         if (query1.value(0).toString().contains("goto"))
        //         {
        //            ui->learnedSoFarListWidget->addItem(query1.value(8).toString());
        //         }
        //         else
        //         {
                    ui->learnedSoFarListWidget->addItem(query1.value(8).toString() + " (" + query1.value(0).toString()  + ")");
        //         }
             }

           }

        }
     }




}

void MainWindow::on_stopLearningPushButton_clicked()
{
 //   ui->startLearningPushButton->show();
    ui->stopLearningPushButton->hide();

    ui->learningStartedLabel->setStyleSheet("QLabel { background-color : red; color : white; qproperty-alignment: AlignCenter;}");
    ui->learningStartedLabel->setText("Learning Stopped");

    ui->alreadyKnowWidget->setEnabled(false);

    if (ui->learnedSoFarListWidget->count() != 0)
    {
        ui->rememberThisPushButton->show();
    }
}


void MainWindow::on_behNameComboBox_editTextChanged(QString )
{
    ui->addPushButton->show();

    ui->deletePushButton->hide();
    ui->changePushButton->hide();
//    qDebug()<<"edit";
}

void MainWindow::on_behNameComboBox_activated(QString behName)
{
   //     qDebug()<<"act";

     ui->addPushButton->hide();

     ui->deletePushButton->show();
     ui->changePushButton->show();

     QSqlQuery query;
     query.prepare("SELECT * FROM Sequences WHERE name = :name AND experimentalLocationId = :locn");
     query.bindValue(":name",behName);
     query.bindValue(":locn", expLocation);

     if (query.exec())
     {
       while (query.next())
       {
          ui->behNameLineEdit->setText(query.value(8).toString());
       }
     }
     else
     {
        QMessageBox msgBox;
        msgBox.setIcon(QMessageBox::Critical);

        msgBox.setText("Database error - can't select from Sequence table!");
        msgBox.exec();

        qCritical("Cannot select: %s (%s)",
                       db.lastError().text().toLatin1().data(),
                       qt_error_string().toLocal8Bit().data());
        return;
     }

      ui->behNameLineEdit->show();
 //    ui->startLearningPushButton->show();
      ui->okPushButton->show();

}

void MainWindow::on_addPushButton_clicked()
{

    QString str = ui->behNameComboBox->currentText();

    str = str.simplified();
    str.replace( " ", "" );

    if (str=="")
    {

       QMessageBox msgBox;
       msgBox.setIcon(QMessageBox::Critical);

       msgBox.setText("You need to provide a name for what you are teaching!");
       msgBox.exec();

   //    ui->behNameComboBox->show();
   //    ui->behNameComboBox->setEnabled(true);
   //    ui->behNameComboBox->setStyleSheet("QComboBox { background-color : white; color : black; }");
         ui->behNameComboBox->setFocus();

   //    ui->startLearningPushButton->hide();

       return;

    }

     str = ui->behNameLineEdit->text();
     str = str.simplified();
     str.replace( " ", "" );

     if (str=="")
     {

        QMessageBox msgBox;
        msgBox.setIcon(QMessageBox::Critical);

        msgBox.setText("You need to provide a long name for what you are teaching!");
        msgBox.exec();

  //      ui->behNameComboBox->show();
  //      ui->behNameComboBox->setEnabled(true);
   //     ui->behNameComboBox->setStyleSheet("QComboBox { background-color : white; color : black; }");
        ui->behNameLineEdit->setFocus();

   //     ui->startLearningPushButton->hide();

        return;

     }



    on_startLearningPushButton_clicked();
 
}

void MainWindow::on_rememberThisPushButton_clicked()
{
    ui->alreadyKnowWidget->setEnabled(true);
//    ui->alreadyKnowWidget->setTabEnabled(0,false);
//    ui->alreadyKnowWidget->setTabEnabled(1,false);
//    ui->alreadyKnowWidget->setTabEnabled(2,false);
//    ui->alreadyKnowWidget->setTabEnabled(3,false);
//    ui->alreadyKnowWidget->setTabEnabled(4,false);
    ui->alreadyKnowWidget->setCurrentIndex(5);
    ui->alreadyKnowWidget->setTabEnabled(5,true);
    ui->startLearningPushButton->hide();
    ui->rememberThisPushButton->hide();
    ui->learningStartedLabel->hide();
    ui->whenPushButton->show();
    ui->whenPushButton_2->hide();
    ui->whatPushButton_2->show();
    ui->whatPushButton->hide();
}

void MainWindow::on_okPushButton_clicked()
{
    int tab = ui->alreadyKnowWidget->currentIndex();
//    qDebug()<<tab;

    for (int i=0;i<7;i++)
    {
 //     ui->alreadyKnowWidget->setTabEnabled(i,false);
    }
    
    if (ui->whenTellMePushButton->isChecked() && !ui->toldFinishedPushButton->isChecked())
    {
       ui->alreadyKnowWidget->setTabEnabled(6,true);
       ui->alreadyKnowWidget->setCurrentIndex(6);
       return;
    }

    if (ui->whenSomethingPushButton->isChecked() && !ui->houseFinishedPushButton->isChecked())
    {
       ui->alreadyKnowWidget->setTabEnabled(7,true);
       ui->alreadyKnowWidget->setCurrentIndex(7);
       return;
    }
    
    if (ui->doingPushButton->isChecked() && !ui->contextFinishedPushButton->isChecked())
    {
       ui->alreadyKnowWidget->setTabEnabled(8,true);
       ui->alreadyKnowWidget->setCurrentIndex(8);
       return;
    }

    if (ui->locationPushButton->isChecked() && !ui->locnFinishedPushButton->isChecked())
    {
       fillLocationWhen();
       ui->alreadyKnowWidget->setTabEnabled(9,true);
       ui->alreadyKnowWidget->setCurrentIndex(9);
       return;
    }

    if (ui->timePushButton->isChecked() && !ui->timeFinishedPushButton->isChecked())
    {
        ui->alreadyKnowWidget->setTabEnabled(10,true);
        ui->alreadyKnowWidget->setCurrentIndex(10);

        return;
    }

    ui->alreadyKnowWidget->setTabEnabled(11,true);
    ui->alreadyKnowWidget->setCurrentIndex(11);



}

void MainWindow::on_alreadyKnowWidget_currentChanged(int index)
{
       qDebug()<<"tab"<<index;

        QSqlQuery query;



        switch (index)
        {


            case 0:
            query.prepare("SELECT * FROM Sequences where (scenario = 'User Generated') and name not like 'reset%' AND experimentalLocationId = '" +  expLocation + "' order by scenarioDescription");
              break;
            case 2:
              query.prepare("SELECT * FROM ActionRules where action LIKE 'speak%' AND experimentalLocationId = '" +  expLocation + "'");
              break;
            case 1:
              {
              query.prepare("select S.scenarioDescription, S.name from Sequences S, ActionRules A where S.name = A.name and A.action LIKE 'base%' AND S.experimentalLocationId = '"\
                      +  expLocation + "' group by S.name");

              ui->moveComboBox->clear();
              ui->moveComboBox->addItem("");

              QSqlQuery locnQuery("SELECT  L1.locationId, L1.where, L2.where, L1.name, IF(STRCMP(L1.name,L2.name),L2.name,''), IF(STRCMP(L2.name,L3.name),L3.name,''), IF(STRCMP(L3.name,L4.name),L4.name,'')\
                    FROM Locations L1,\
                         Locations L2,\
                         Locations L3,\
                         Locations L4\
                    WHERE L2.locationId = L1.where\
                      AND L3.locationId = L2.where\
                      AND L4.locationId = L3.where\
                      AND L1.validRobotLocation = 1\
                      AND (L1.locationID < 100 OR L1.locationID = 999)\
                        ORDER BY L1.locationId");



                  QString q1, q2, q3;

                  q1 = q2 = q3 = "";

                  while(locnQuery.next())
                  {
                      q1 = q2 = q3 = "";

                      if ( locnQuery.value(4).toString() != "")
                      {
                          if ( locnQuery.value(1).toInt() == 0)
                          {
                              q1 = "";
                          }
                          else
                          {
                              q1 = " in the " +   locnQuery.value(4).toString();
                          }
                      }

                      if (locnQuery.value(0).toInt() != 0 ) // && locnQuery.value(0).toInt() != 999 )
                      {
                          ui->moveComboBox->addItem(locnQuery.value(3).toString() + q1 + " ::" + locnQuery.value(0).toString() + "::");
                      }
                  }

                  break; } // index 2
        case 3:

           query.prepare("select S.scenarioDescription, S.name from Sequences S, ActionRules A  where S.name = A.name and \
                                 (A.action LIKE 'tray%' or A.action LIKE 'torso%') AND S.experimentalLocationId = '" +  expLocation + "'");
           break;     // index 3

        case 8:
               ui->contextComboBox->clear();
               query.prepare("SELECT * FROM Sensors where sensorId > 899 and sensorId < 1000");
               break;     // index 8


          }


       QStringListModel *model = new QStringListModel();
       QStringList list;
       QString raText;


       if (query.exec())
       {
          while (query.next())
          {
                switch (index)
                {
                    case 0:

                         list << query.value(8).toString() + " (" + query.value(0).toString() + ")";

                         break;

                     case 2:

                         list << "say \"" + query.value(7).toString().section(",", 2, 2) + "\"";

                        break;

                     case 1:

                         list << query.value(0).toString() + " (" + query.value(1).toString() + ")";

                        break;

                     case 3:

                         list << query.value(0).toString() + " (" + query.value(1).toString() + ")";

                         break;

                     case 8:

                         ui->contextComboBox->addItem(query.value(3).toString() + " (context" + query.value(0).toString() + ")");

                         break;

                    }

                    list.removeDuplicates();
          }

          model->setStringList(list);
      }

       switch (index)
       {
           case 0:
             ui->allKnowHowListView->setModel( model );
             break;

           case 2:
             ui->speakKnowHowListView->setModel( model );
             break;

           case 1:
             ui->moveKnowHowListView->setModel( model );
             break;

           case 3:
             ui->trayTorsoKnowHowListView->setModel( model );
             break;


       }
}

void MainWindow::on_everythingLearnItPushButton_clicked()
{
    if (ui->allKnowHowListView->currentIndex().row() == -1)
    {
        QMessageBox msgBox;
        msgBox.setIcon(QMessageBox::Warning);

        msgBox.setText("You need to click above on which task you want the robot to learn!");
        msgBox.exec();

        return;
    }

    QModelIndex index =  ui->allKnowHowListView->currentIndex();
    QVariant target = index.data().toString();
    QString learntItem = target.toString();
    //.section("(", 1, 1);
    //learntItem.remove(")");

    addToLearningList(learntItem);

}

void MainWindow::addToLearningList(QString learntItem)
{
//    qDebug()<<learntItem;

//    ui->learnedSoFarListView->setModel(learnedModel);

    ui->learnedSoFarListWidget->addItem(learntItem);

}




void MainWindow::on_speakLearnItPushButton_clicked()
{

   QString str = ui->speakLineEdit->text();
   str = str.simplified();
 //  str.replace( " ", "" );
   str.replace( ",", " " );

   if (ui->speakKnowHowListView->currentIndex().row() == -1 && str=="")
   {
        QMessageBox msgBox;
        msgBox.setIcon(QMessageBox::Warning);

        msgBox.setText("You need to click above on which task you want the robot to learn or enter some text for what you want it to say!");
        msgBox.exec();

        return;
    }

    QString learntItem;
 //   str = ui->speakLineEdit->text();

    if (str != "")
    {
       learntItem = "say \"" + str + "\"";
    }
    else
    {
        QModelIndex index =  ui->speakKnowHowListView->currentIndex();
        QVariant target = index.data().toString();
        learntItem = target.toString();
    }


    addToLearningList(learntItem);
}



void MainWindow::on_speakLineEdit_textChanged(QString )
{
        ui->speakKnowHowListView->clearSelection();

}


void MainWindow::on_moveLearnItPushButton_clicked()
{
    QString str = ui->moveComboBox->currentText();
    str = str.simplified();
    str.replace( " ", "" );

    if (ui->moveKnowHowListView->currentIndex().row() == -1 && str=="")
    {
         QMessageBox msgBox;
         msgBox.setIcon(QMessageBox::Warning);

         msgBox.setText("You need to click above on which task you want the robot to learn or choose where you want it to go!");
         msgBox.exec();

         return;
     }


    QString learntItem;

    if (str != "")
    {
        learntItem = "Send the robot to the " + str + ", lowering tray if possible (goto" + str + ")";
    }
    else
    {
        QModelIndex index =  ui->moveKnowHowListView->currentIndex();
        QVariant target = index.data().toString();
        learntItem = target.toString();
                     //section("(", 1, 1);
        //learntItem.remove(")");
    }
    addToLearningList(learntItem);


}



void MainWindow::on_speakKnowHowListView_clicked(QModelIndex index)
{
    ui->speakLineEdit->clear();
}

void MainWindow::on_moveComboBox_activated(QString )
{
    ui->moveKnowHowListView->clearSelection();
}



void MainWindow::on_moveKnowHowListView_clicked(QModelIndex index)
{
    ui->moveComboBox->setCurrentIndex(0);
}





void MainWindow::on_whenTellMePushButton_clicked(bool checked)
{
   if (checked)
   {
     ui->whenTellMePushButton->setStyleSheet("QPushButton { background-color : green; color : white}");

     ui->whenSomethingPushButton->setChecked(false);
     on_whenSomethingPushButton_clicked(false);

     ui->doingPushButton->setChecked(false);
     on_doingPushButton_clicked(false);

     ui->locationPushButton->setChecked(false);
     on_locationPushButton_clicked(false);

     ui->timePushButton->setChecked(false);
     on_timePushButton_clicked(false);
   }
   else
   {
     ui->whenTellMePushButton->setStyleSheet("QPushButton { background-color : qlineargradient(x1: 0, y1: 0, x2: 0, y2: 1, stop: 0 #f6f7fa, stop: 1 #dadbde);color : black}");
   }

   checkOKVisibleFlag();


}

void MainWindow::on_whenSomethingPushButton_clicked(bool checked)
{
    if (checked)
    {
      ui->whenSomethingPushButton->setStyleSheet("QPushButton { background-color : green; color : white}");
      ui->whenTellMePushButton->setChecked(false);
      on_whenTellMePushButton_clicked(false);
    }
    else
    {
      ui->whenSomethingPushButton->setStyleSheet("QPushButton { background-color : qlineargradient(x1: 0, y1: 0, x2: 0, y2: 1, stop: 0 #f6f7fa, stop: 1 #dadbde);color : black}");
    }

    checkOKVisibleFlag();

}



void MainWindow::on_doingPushButton_clicked(bool checked)
{
    if (checked)
    {
      ui->doingPushButton->setStyleSheet("QPushButton { background-color : green; color : white}");
      ui->whenTellMePushButton->setChecked(false);
      on_whenTellMePushButton_clicked(false);
    }
    else
    {
      ui->doingPushButton->setStyleSheet("QPushButton { background-color : qlineargradient(x1: 0, y1: 0, x2: 0, y2: 1, stop: 0 #f6f7fa, stop: 1 #dadbde);color : black}");
    }

    checkOKVisibleFlag();

}

void MainWindow::on_locationPushButton_clicked(bool checked)
{
    if (checked)
    {
      ui->locationPushButton->setStyleSheet("QPushButton { background-color : green; color : white}");
      ui->whenTellMePushButton->setChecked(false);
      on_whenTellMePushButton_clicked(false);
    }
    else
    {
      ui->locationPushButton->setStyleSheet("QPushButton { background-color : qlineargradient(x1: 0, y1: 0, x2: 0, y2: 1, stop: 0 #f6f7fa, stop: 1 #dadbde);color : black}");
    }

    checkOKVisibleFlag();
}

void MainWindow::on_timePushButton_clicked(bool checked)
{
    if (checked)
    {
      ui->timePushButton->setStyleSheet("QPushButton { background-color : green; color : white}");
      ui->whenTellMePushButton->setChecked(false);
      on_whenTellMePushButton_clicked(false);
    }
    else
    {
      ui->timePushButton->setStyleSheet("QPushButton { background-color : qlineargradient(x1: 0, y1: 0, x2: 0, y2: 1, stop: 0 #f6f7fa, stop: 1 #dadbde);color : black}");
    }

    checkOKVisibleFlag();
}


void MainWindow::checkOKVisibleFlag()
{

    if (ui->whenSomethingPushButton->isChecked() ||
        ui->doingPushButton->isChecked()         ||
        ui->locationPushButton->isChecked()      ||
        ui->whenTellMePushButton->isChecked()    ||
        ui->timePushButton->isChecked())
    {
        ui->okPushButton->show();
    }
    else
    {
        ui->okPushButton->hide();
    }
}




void MainWindow::fillFinalView()
{


    QStringListModel *model = new QStringListModel();
    QStringList displayList;

    displayList << "";
    displayList << "You taught me to: " ;
 //   displayList << "";
    displayList << "         " + ui->behNameLineEdit->text() + " (" + ui->behNameComboBox->currentText() + ")";
  //  displayList << "";


    learnedList.clear();

    for (int i=0; i < ui->learnedSoFarListWidget->count();i++)
    {
       learnedList.append(ui->learnedSoFarListWidget->item(i)->text());
    }


    QStringList result = learnedList.filter("When:");
    if (result.size() > 0)
    {
        displayList << "When: ";

       result.replaceInStrings("When: ","         ");

       for (int i = 0; i < result.size(); ++i)
       {
          displayList << result.at(i);

       }
    }

    displayList<<"Then: ";

    for (int i = 0; i < learnedList.size(); ++i)
    {
        if (!learnedList.at(i).contains("When:"))
        {
            displayList << "        " + learnedList.at(i);
        }
    }
    

    model->setStringList(displayList);
    ui->finalListView->setModel( model );

    ui->whenPushButton_2->show();
    ui->whenPushButton->hide();
    ui->finishedB->show();
    ui->finsihedB2->hide();


}

void MainWindow::fillLocationWhen()
{


    ui->userLocnComboBox->clear();
    ui->userLocnComboBox->addItem("");
// hard coded for UH!!
    QSqlQuery query("SELECT  L1.locationId, L1.where, L2.where, L1.name, IF(STRCMP(L1.name,L2.name),L2.name,''), IF(STRCMP(L2.name,L3.name),L3.name,''), IF(STRCMP(L3.name,L4.name),L4.name,'')\
                FROM Locations L1,\
                     Locations L2,\
                     Locations L3,\
                     Locations L4\
                WHERE L2.locationId = L1.where\
                  AND L3.locationId = L2.where\
                  AND L4.locationId = L3.where\
                  AND L1.validUserLocation = 1\
                  AND L1.locationId < 100\
                ORDER BY L1.locationId");

    QString q1, q2, q3;

    q1 = q2 = q3 = "";

    while(query.next())
    {
 //     q1 = q2 = q3 = "";

 //     if ( query.value(4).toString() != "")
 //     {
  //      if ( query.value(1) == 0)
 //       {
 //           q1 = "";
 //       }
 //       else
 //       {
 //           q1 = " in the " +   query.value(4).toString();
 //       }
 //     }

 //     q1 = q1 + " (" + query.value(0).toString() + ")";

      q1 = " (" + query.value(0).toString() + ")";

      if (query.value(0).toString() != "0" && query.value(0).toString() != "999")
      {
         ui->userLocnComboBox->addItem(query.value(3).toString() + q1);
      }
   }

    ui->andRobotLocnComboBox->clear();
    ui->andRobotLocnComboBox->addItem("");

    query.clear();

    query.prepare("SELECT  L1.locationId, L1.where, L2.where, L1.name, IF(STRCMP(L1.name,L2.name),L2.name,''), IF(STRCMP(L2.name,L3.name),L3.name,''), IF(STRCMP(L3.name,L4.name),L4.name,'')\
                FROM Locations L1,\
                     Locations L2,\
                     Locations L3,\
                     Locations L4\
                WHERE L2.locationId = L1.where\
                  AND L3.locationId = L2.where\
                  AND L4.locationId = L3.where\
                  AND L1.validRobotLocation = 1\
                  AND L1.locationID < 100\
                ORDER BY L1.locationId");

    query.exec();

    q1 = q2 = q3 = "";

    while(query.next())
    {

      q1 = " (" + query.value(0).toString() + ")";

      if (query.value(0).toString() != "0" && query.value(0).toString() != "999")
      {
         ui->andRobotLocnComboBox->addItem(query.value(3).toString() + q1);
      }
   }

    QStringListModel *model = new QStringListModel();
    QStringList locnList;
    QString raText;

    query.clear();
    query.prepare("SELECT ruleActiontext FROM ActionRules where  ruleActionText like 'When: The user is in%' or ruleActionText like 'When: The robot is in%' ");

    if (query.exec())
    {
       while (query.next())
       {


           locnList << query.value(0).toString();

           locnList.removeDuplicates();
       }

       model->setStringList(locnList);
       ui->locationListView->setModel( model );
   }





}


void MainWindow::on_atCheckBox_toggled(bool checked)
{
    if (checked)
    {
        ui->betweenCheckBox->setChecked(false);
    }
}

void MainWindow::on_betweenCheckBox_toggled(bool checked)
{
    if (checked)
    {
        ui->atCheckBox->setChecked(false);
    }

}

void MainWindow::on_timeLearnItPushButton_clicked()
{
    bool addedToLearn = false;

    if (ui->atCheckBox->isChecked())
    {
        addToLearningList("When:  At " + ui->AtTimeEdit->time().toString("hh:mm"));
        addedToLearn = true;
        ui->atCheckBox->setChecked(false);
    }

    if (ui->betweenCheckBox->isChecked())
    {
        if (ui->betweenFromTimeEdit->time() >= ui->betweenToTimeEdit->time())
        {
            QMessageBox msgBox;
            msgBox.setIcon(QMessageBox::Warning);

            msgBox.setText("The second time must be after the first time!");
            msgBox.exec();

           return;
        }

        addToLearningList("When: Between " + ui->betweenFromTimeEdit->time().toString("hh:mm") + " and " + ui->betweenToTimeEdit->time().toString("hh:mm"));
        addedToLearn = true;
        ui->betweenCheckBox->setChecked(false);
    }

    bool days = false;
    QString daysString="";



    if (ui->mondayCheckBox->isChecked())
    {
       daysString += "Mon";
       days = true;
    }

    if (ui->tuesdayCheckBox->isChecked())
    {
       if (days) daysString+=",";

       daysString += "Tue";
       days = true;
    }
    if (ui->wednesdayCheckBox->isChecked())
    {
       if (days) daysString+=",";
       daysString += "Wed";
       days = true;
    }
    if (ui->ThursdayCheckBox->isChecked())
    {
       if (days) daysString+=",";
       daysString += "Thu";
       days = true;
    }
    if (ui->fridayCheckBox->isChecked())
    {
       if (days) daysString+=",";
       daysString += "Fri";
       days = true;
    }
    if (ui->saturdayCheckBox->isChecked())
    {
       if (days) daysString+=",";
       daysString += "Sat";
       days = true;
    }
    if (ui->sundayCheckBox->isChecked())
    {
       if (days) daysString+=",";
       daysString += "Sun";
       days = true;
    }

    if (ui->everyNdaysSpinBox->value() > 0)
    {
        days = false;

        QString v;
        addToLearningList("When: Repeat every: " + v.setNum(ui->everyNdaysSpinBox->value()) + " days"  );
        addedToLearn = true;
    }

    if (days)
    {
        addToLearningList("When: On the following days:" + daysString);
        addedToLearn = true;

        ui->mondayCheckBox->setChecked(false);
        ui->tuesdayCheckBox->setChecked(false);
        ui->wednesdayCheckBox->setChecked(false);
        ui->ThursdayCheckBox->setChecked(false);
        ui->fridayCheckBox->setChecked(false);
        ui->saturdayCheckBox->setChecked(false);
        ui->sundayCheckBox->setChecked(false);
    }

    if (ui->everyNHoursSpinBox->value() > 0)
    {
        QString v;
        v.setNum(ui->everyNHoursSpinBox->value());
        addToLearningList("When: Repeat starting at: " + ui->AtTimeEdit->time().toString("hh:mm") + " then every: " +  v + " hours");
        addedToLearn = true;

    }


    if (ui->resetSpinBox->value() > 0)
    {
        int v;
        v = ui->resetSpinBox->value();

        int days = v / 86400;
        v = v%86400;

        int hours=v/3600;
        v = v%3600;

        int min=v/60;
        v = v%60;

        int sec = v;

        QString txt;

        if (days > 0)
        {
            QString d;
            d.setNum(days);
            txt += d + " days ";
        }

        if (hours > 0)
        {
            QString h;
            h.setNum(hours);
            txt += h + " hours ";
        }

        if (min > 0)
        {
            QString m;
            m.setNum(min);
            txt += m + " minutes ";
        }

        if (sec > 0)
        {
            QString s;
            s.setNum(sec);
            txt += s + " seconds ";
        }

        addToLearningList("When: Remind me/Reset after " +  txt);
        addedToLearn = true;
    }



    if (!addedToLearn)
    {
        QMessageBox msgBox;
        msgBox.setIcon(QMessageBox::Warning);

        msgBox.setText("You need to choose something for the robot to learn!");
        msgBox.exec();

       return;

    }

    ui->timeFinishedPushButton->show();

    ui->everyNdaysSpinBox->setValue(0);
    ui->everyNHoursSpinBox->setValue(0);
}






void MainWindow::on_locationLearnItPushButton_clicked()
{

    QString strRobot = ui->andRobotLocnComboBox->currentText();
    QString strUser  = ui->userLocnComboBox->currentText();

    if (ui->locationListView->currentIndex().row() == -1 && strRobot == "" && strUser == "")
    {
         QMessageBox msgBox;
         msgBox.setIcon(QMessageBox::Warning);

         msgBox.setText("You need to click above or choose in which location you want it to happen!");
         msgBox.exec();

         return;
     }

    QString learntItem;

    if (strRobot != "" || strUser != "")
    {
      if (strRobot != "")
      {
        learntItem = "When: The Robot is in the " + ui->andRobotLocnComboBox->currentText();
        addToLearningList(learntItem);
      }

      if (strUser != "")
      {
        learntItem = "When: The User is in the " + ui->userLocnComboBox->currentText();
        addToLearningList(learntItem);
      }
    }
    else
    {
      if (ui->locationListView->currentIndex().row() != -1)
      {
        QModelIndex index =  ui->locationListView->currentIndex();
        QVariant target = index.data().toString();
        learntItem = target.toString();
      }

      addToLearningList(learntItem);
    }

    ui->locnFinishedPushButton->show();


}



void MainWindow::on_userLocnComboBox_activated(QString )
{
        ui->locationListView->clearSelection();
}

void MainWindow::on_andRobotLocnComboBox_activated(QString )
{
        ui->locationListView->clearSelection();
}

void MainWindow::on_locationListView_clicked(QModelIndex index)
{
        ui->andRobotLocnComboBox->setCurrentIndex(0);
        ui->userLocnComboBox->setCurrentIndex(0);
}

void MainWindow::on_locnFinishedPushButton_clicked()
{
    on_okPushButton_clicked();
}

void MainWindow::on_timeFinishedPushButton_clicked()
{
     on_okPushButton_clicked();
}

void MainWindow::on_houseFinishedPushButton_clicked()
{
     on_okPushButton_clicked();
}


void MainWindow::on_diningLearnItPushButton_clicked()
{
    bool addedToLearn = false;

    QString txt;

    if (ui->sittingTableCheckBox->isChecked())
    {

        txt = createForWithinMsg("sitTable",
                           ui->happeningSpinBox->value(),
                           ui->hasHappendedSpinBox->value(),
                           "When: Someone is sitting at the table ",
                           "When: Someone has been sitting at the table for ",
                           "When: Someone was sitting at the table within the last ");

        addToLearningList(txt);
        addedToLearn = true;
        ui->sittingTableCheckBox->setChecked(false);

    }


    if (ui->bigCupboardCheckBox->isChecked())
    {

        txt = createForWithinMsg("bigCbdOpen",
                           ui->happeningSpinBox->value(),
                           ui->hasHappendedSpinBox->value(),
                           "When: The big cupboard doors or drawers are open ",
                           "When: The big cupboard doors or drawers have been open for ",
                           "When: The big cupboard doors or drawers were opened within the last ");

        addToLearningList(txt);
        addedToLearn = true;
        ui->bigCupboardCheckBox->setChecked(false);

    }

    if (ui->smallCupboardCheckBox->isChecked())
    {

        txt = createForWithinMsg("smallCbdOpen",
                           ui->happeningSpinBox->value(),
                           ui->hasHappendedSpinBox->value(),
                           "When: The small cupboard doors or drawers are open ",
                           "When: The small cupboard doors or drawers have been open for ",
                           "When: The small cupboard doors or drawers were opened within the last ");

        addToLearningList(txt);
        addedToLearn = true;
        ui->smallCupboardCheckBox->setChecked(false);

    }

    ui->happeningSpinBox->setValue(0);
    ui->hasHappendedSpinBox->setValue(0);
    ui->happeningSpinBox->setEnabled(false);
    ui->hasHappendedSpinBox->setEnabled(false);

    if (!addedToLearn)
    {
        QMessageBox msgBox;
        msgBox.setIcon(QMessageBox::Warning);

        msgBox.setText("You need to choose something for the robot to learn!");
        msgBox.exec();

       return;

    }

    ui->houseFinishedPushButton->show();
}

void MainWindow::on_livingLearnItPushButton_clicked()    // living room tab
{
    bool addedToLearn = false;

    QString txt;

    if (ui->sittingSofaCheckBox->isChecked())
    {

        txt = createForWithinMsg("sitSofa",
                           ui->happeningSpinBox->value(),
                           ui->hasHappendedSpinBox->value(),
                           "When: Someone is sitting on the sofa ",
                           "When: Someone has been sitting on the sofa for ",
                           "When: Someone was sitting on the sofa within the last ");

        addToLearningList(txt);
        addedToLearn = true;
        ui->sittingSofaCheckBox->setChecked(false);

    }

    if (ui->TVOncheckBox->isChecked())
    {
        txt = createForWithinMsg("TVOn",
                           ui->happeningSpinBox->value(),
                           ui->hasHappendedSpinBox->value(),
                           "When: The TV is on ",
                           "When: The TV has been on for ",
                           "When: The TV was on within the last ");

        addToLearningList(txt);
        addedToLearn = true;
        ui->TVOncheckBox->setChecked(false);
    }


    ui->happeningSpinBox->setValue(0);
    ui->hasHappendedSpinBox->setValue(0);
    ui->happeningSpinBox->setEnabled(false);
    ui->hasHappendedSpinBox->setEnabled(false);

    if (!addedToLearn)
    {
        QMessageBox msgBox;
        msgBox.setIcon(QMessageBox::Warning);

        msgBox.setText("You need to choose something for the robot to learn!");
        msgBox.exec();

       return;

    }


    ui->houseFinishedPushButton->show();
}

void MainWindow::on_kitchenLearnItPushButton_clicked()             // kitchen tab
{
    bool addedToLearn = false;

    QString txt;

    if (ui->kitechCupboadsCheckBox->isChecked())
    {

        txt = createForWithinMsg("kitCpdOpen",
                           ui->happeningSpinBox->value(),
                           ui->hasHappendedSpinBox->value(),
                           "When: The kitchen drawers or cupboards are open ",
                           "When: The kitchen drawers or cupboards have been open for ",
                           "When: The kitchen drawers or cupboards have been opened within the last ");

         addToLearningList(txt);
         addedToLearn = true;
         ui->kitechCupboadsCheckBox->setChecked(false);
     }


    if (ui->kitchenTapsCheckBox->isChecked())
    {

        txt = createForWithinMsg("KitTapsOn",
                           ui->happeningSpinBox->value(),
                           ui->hasHappendedSpinBox->value(),
                           "When: The kitchen taps are running " ,
                           "When: The kitchen taps have been running for ",
                           "When: The kitchen taps were running within the last ");

        addToLearningList(txt);
        addedToLearn = true;
        ui->kitchenTapsCheckBox->setChecked(false);
    }

    ui->happeningSpinBox->setValue(0);
    ui->hasHappendedSpinBox->setValue(0);
    ui->happeningSpinBox->setEnabled(false);
    ui->hasHappendedSpinBox->setEnabled(false);

    if (!addedToLearn)
    {
        QMessageBox msgBox;
        msgBox.setIcon(QMessageBox::Warning);

        msgBox.setText("You need to choose something for the robot to learn!");
        msgBox.exec();

       return;

    }

    ui->houseFinishedPushButton->show();
}

void MainWindow::on_ElectricalLearnItPushButton_clicked()
{
    bool addedToLearn = false;
    QString txt;

    if (ui->microwaveCheckBox->isChecked())
    {
        txt = createForWithinMsg("microOn",
                           ui->happeningSpinBox->value(),
                           ui->hasHappendedSpinBox->value(),
                           "When: The microwave, cooker or toaster is on  ",
                           "When: The microwave, cooker or toaster have been on for ",
                           "When: The microwave, cooker or toaster were on within the last ");

        addToLearningList(txt);
        addedToLearn = true;
        ui->microwaveCheckBox->setChecked(false);
    }

    if (ui->fridgeCheckBox->isChecked())
    {

        txt = createForWithinMsg("frgOn",
                           ui->happeningSpinBox->value(),
                           ui->hasHappendedSpinBox->value(),
                           "When: The fridge is open, kettle or dishwasher are on ",
                           "When: The fridge has been open or the kettle or dishwasher have been on for ",
                           "When: The fridge was open, kettle or dishwasher were on within the last ");

        addToLearningList(txt);
        addedToLearn = true;
        ui->fridgeCheckBox->setChecked(false);
    }

    if (ui->doorbellCheckBox->isChecked())
    {
        addToLearningList("When: The doorbell has rung (DoorbellRang)");
        addedToLearn = true;
        ui->doorbellCheckBox->setChecked(false);
    }

    ui->happeningSpinBox->setValue(0);
    ui->hasHappendedSpinBox->setValue(0);
    ui->happeningSpinBox->setEnabled(false);
    ui->hasHappendedSpinBox->setEnabled(false);

    if (!addedToLearn)
    {
        QMessageBox msgBox;
        msgBox.setIcon(QMessageBox::Warning);

        msgBox.setText("You need to choose something for the robot to learn!");
        msgBox.exec();

       return;

    }

    ui->houseFinishedPushButton->show();
}

void MainWindow::on_BathroomLearnItPushButton_clicked()
{
    bool addedToLearn = false;

    QString txt;

    if (ui->bathTapsCheckBox->isChecked())
    {

        txt = createForWithinMsg("bathTapsOn",
                           ui->happeningSpinBox->value(),
                           ui->hasHappendedSpinBox->value(),
                           "When: The bathroom taps are running " ,
                           "When: The bathroom taps have been running for ",
                           "When: The bathroom taps were running within the last ");

        addToLearningList(txt);
        addedToLearn = true;
        ui->bathTapsCheckBox->setChecked(false);
    }

    if (ui->bathDoorCheckBox->isChecked())
    {

        txt = createForWithinMsg("bathDoorClosed",
                           ui->happeningSpinBox->value(),
                           ui->hasHappendedSpinBox->value(),
                           "When: The bathroom door is closed " ,
                           "When: The bathroom door has been closed for ",
                           "When: The bathroom door was closed within the last ");

        addToLearningList(txt);
        addedToLearn = true;
        ui->bathDoorCheckBox->setChecked(false);
    }

    if (ui->bathFlushCheckBox->isChecked())
    {

        txt = createForWithinMsg("toiletFlush",
                           ui->happeningSpinBox->value(),
                           ui->hasHappendedSpinBox->value(),
                           "When: The bathroom toilet is flushing " ,
                           "When: The bathroom toilet has been flushing for ",
                           "When: The bathroom toilet was flushed within the last ");

        addToLearningList(txt);
        addedToLearn = true;
        ui->bathFlushCheckBox->setChecked(false);
    }


    ui->happeningSpinBox->setValue(0);
    ui->hasHappendedSpinBox->setValue(0);
    ui->happeningSpinBox->setEnabled(false);
    ui->hasHappendedSpinBox->setEnabled(false);

    if (!addedToLearn)
    {
        QMessageBox msgBox;
        msgBox.setIcon(QMessageBox::Warning);

        msgBox.setText("You need to choose something for the robot to learn!");
        msgBox.exec();

       return;

    }

    ui->houseFinishedPushButton->show();
}

void MainWindow::on_toldFinishedPushButton_clicked()
{
        addToLearningList("When: You click on this command on the tablet (userInit)");
        on_okPushButton_clicked();
}

void MainWindow::on_contextLearnItPushButton_clicked()
{
    QString txt;

    QString context = ui->contextComboBox->currentText().section('(',1,1).section(')',0,0);
    qDebug()<<context;
    QString activity = ui->contextComboBox->currentText().section('(',0,0);
    qDebug()<<activity;

    txt = createForWithinMsg(context,
                       ui->contextHappeningSpinBox->value(),
                       ui->contextHasHappendedSpinBox->value(),
                       "When: " + activity +" is active " ,
                       "When: " + activity +" has been active for " ,
                       "When: " + activity +" was active within the last ");


    addToLearningList(txt);


    ui->contextFinishedPushButton->show();
}

void MainWindow::on_contextFinishedPushButton_clicked()
{
     on_okPushButton_clicked();
}

void MainWindow::on_trayLearnItPushButton_clicked()
{

    if (ui->trayTorsoKnowHowListView->currentIndex().row() == -1)
    {
         QMessageBox msgBox;
         msgBox.setIcon(QMessageBox::Warning);

         msgBox.setText("You need to click above on which task you want the robot to learn!");
         msgBox.exec();

         return;
     }


    QString learntItem;

    QModelIndex index =  ui->trayTorsoKnowHowListView->currentIndex();
    QVariant target = index.data().toString();
    learntItem = target.toString();

    addToLearningList(learntItem);
}


void MainWindow::on_mondayCheckBox_toggled(bool checked)
{
    if (checked)
    {
        ui->everyNdaysSpinBox->setValue(0);
    }
}

void MainWindow::on_tuesdayCheckBox_toggled(bool checked)
{
    if (checked)
    {
        ui->everyNdaysSpinBox->setValue(0);
    }
}

void MainWindow::on_ThursdayCheckBox_toggled(bool checked)
{
    if (checked)
    {
        ui->everyNdaysSpinBox->setValue(0);
    }
}

void MainWindow::on_wednesdayCheckBox_toggled(bool checked)
{
    if (checked)
    {
        ui->everyNdaysSpinBox->setValue(0);
    }
}

void MainWindow::on_fridayCheckBox_toggled(bool checked)
{
    if (checked)
    {
        ui->everyNdaysSpinBox->setValue(0);
    }
}

void MainWindow::on_saturdayCheckBox_toggled(bool checked)
{
    if (checked)
    {
        ui->everyNdaysSpinBox->setValue(0);
    }
}

void MainWindow::on_sundayCheckBox_toggled(bool checked)
{
    if (checked)
    {
        ui->everyNdaysSpinBox->setValue(0);
    }
}

void MainWindow::on_everyNdaysSpinBox_valueChanged(int val)
{
    if (val > 0)
    {
       ui->everyNHoursSpinBox->setValue(0);
   //    ui->resetSpinBox->setValue(val * 3600 * 24);
       ui->resetSpinBox->setEnabled(false);
   //    ui->alwaysCheckBox->setEnabled(false);
       ui->alwaysCheckBox->setChecked(true);
    }

    if (val == 0)
    {
        ui->resetSpinBox->setValue(60);
        ui->resetSpinBox->setEnabled(true);
        ui->alwaysCheckBox->setEnabled(true);
    }
}

void MainWindow::on_learnedSoFarListWidget_doubleClicked(QModelIndex index)
{

    int ret = QMessageBox::warning(this, tr("Teach Me Show Me"),
                                    tr("Do you really want to delete this?"),
                                    QMessageBox::Yes | QMessageBox::No,
                                    QMessageBox::No);

     if (ret == QMessageBox::Yes)
     {
        ui->learnedSoFarListWidget->takeItem(ui->learnedSoFarListWidget->currentRow());
     }

     if (ui->alreadyKnowWidget->currentIndex()==11)
     {
       fillFinalView();
     }

}


void MainWindow::on_finalRememberPushButton_clicked()
{
    fillFinalView();

    deleteExistingBehaviour(ui->behNameComboBox->currentText());

    int ruleCount = 0;
    QSqlQuery query;
    QString sequenceName = ui->behNameComboBox->currentText();

    QStringList result = learnedList.filter("When:");
    if (result.size() > 0)
    {
       for (int i = 0; i < result.size(); ++i)
       {
           qDebug() << "learned list rules:::" << result.at(i);

          // process for sql

           QString cmd = result.at(i);
           QString ruletext;
           QString actionText;
           QString command;

           // get the bit in brackets


           command = cmd.section("(",1,1);
           command = command.section(")",0,0);

           qDebug()<<"learned list      :: " <<command;


           QString contextCmd = command.section("For:",0,0);  // context has two instances
           contextCmd = contextCmd.section("Within:",0,0);  // context has two instances

           qDebug()<<"learned list      :: " <<command << " " << contextCmd;

           // save the complete text

           ruletext = cmd;

           // sensors and context - generate the sql


           generateForWithinSQL(command, "sitTable", "SELECT * FROM Sensors WHERE (sensorId = 20 or sensorId = 21) and value = 0",actionText);

           generateForWithinSQL(command, "bigCbdOpen", "SELECT * FROM Sensors WHERE (sensorId = 24 or sensorId = 25) and value = 1",actionText);

           generateForWithinSQL(command, "smallCbdOpen", "SELECT * FROM Sensors WHERE (sensorId = 26 or sensorId = 27 or sensorId = 28 or sensorId = 29  or sensorId = 30) and value = 1",actionText);

           generateForWithinSQL(command, "sitSofa", "SELECT * FROM Sensors WHERE (sensorId = 15 or sensorId = 16 or sensorId = 17 or sensorId = 18  or sensorId = 19) and value = 0",actionText);

           generateForWithinSQL(command, "TVOn", "SELECT * FROM Sensors WHERE sensorId = 49 AND status = 'On'",actionText);

           generateForWithinSQL(command, "kitCpdOpen", "SELECT * FROM Sensors WHERE (sensorId = 3 or sensorId = 4 or sensorId = 5 or sensorId = 6  or sensorId = 7 or sensorId = 8 or sensorId = 9  or sensorId = 10) and value = 1",actionText);

           generateForWithinSQL(command, "KitTapsOn", "SELECT * FROM Sensors WHERE (sensorId = 1 or sensorId = 2) AND status = 'On'",actionText);

           generateForWithinSQL(command, "microOn", "SELECT * FROM Sensors WHERE (sensorId = 54 or sensorId = 44 or sensorId = 56) AND status = 'On'",actionText);

           generateForWithinSQL(command, "frgOn",  "SELECT * FROM Sensors WHERE (sensorId = 50 or sensorId = 51 or sensorId = 55) AND status = 'On'",actionText);

           if (command == "DoorbellRang")  // within last 10 seconds
           {
               ruletext = cmd;
               actionText  = "SELECT * FROM Sensors WHERE sensorId = 59 AND lastActiveValue > 0 and lastUpdate+INTERVAL 10 SECOND >= NOW()";
           }

           generateForWithinSQL(command, "bathTapsOn", "SELECT * FROM Sensors WHERE (sensorId = 11 or sensorId = 12) AND status = 'On'",actionText);

           generateForWithinSQL(command, "bathDoorClosed", "SELECT * FROM Sensors WHERE sensorId = 13 AND value = 0",actionText);

           generateForWithinSQL(command, "toiletFlush", "SELECT * FROM Sensors WHERE sensorId = 14 AND value = 1",actionText);

           if (command.contains("context"))
           {
              generateForWithinSQL(command, contextCmd, "SELECT * FROM Sensors WHERE sensorId = " + contextCmd.right(3) + " AND value  = 1",actionText);
           }


           // locations

           if (cmd.mid(6,14)  == "The User is in")
           {
               actionText  = "call spCheckUserLocation(" + defaultUser + "," + command + ",@row)";
           }

           if (cmd.mid(6,15)  == "The Robot is in")
           {
               actionText  = "call spCheckRobotLocation(" + defaultRobotStr + "," + command + ",@row)";
           }



           if (cmd.mid(6,4)  == " At ")
           {
               actionText  = "CALL spBetweenTimeCheck('"  + cmd.mid(10,5) + ":00','23:59:59')";
           }


           if (cmd.mid(6,7)  == "Between")
           {
               actionText  = "CALL spBetweenTimeCheck('"  + cmd.mid(14,5) + ":00','" + cmd.mid(24,5) + ":00')";
           }


           if (cmd.mid(6,9)  == "Remind me")
           {
               actionText  = "select 1";              // this only serves as a placeholder always returning true, the reset is used to create a seperate behaviour
               if (cmd.contains("minutes"))
               {
                    QString txt = cmd.section("after",1,1);
                    txt = txt.section("minutes",0,0);
                    int t = txt.toInt();
                    t = t * 60;
                    globalReset = t;
               }
               if (cmd.contains("hours"))
               {
                    QString txt = cmd.section("after",1,1);
                    txt = txt.section("hours",0,0);
                    int t = txt.toInt();
                    t = t * 60 * 60;
                    globalReset = t;
               }
               if (cmd.contains("days"))
               {
                    QString txt = cmd.section("after",1,1);
                    txt = txt.section("days",0,0);
                    int t = txt.toInt();
                    t = t * 60 * 60 * 24;
                    globalReset = t;
               }
               if (cmd.contains("seconds"))
               {
                    QString txt = cmd.section("after",1,1);
                    txt = txt.section("seconds",0,0);
                    int t = txt.toInt();
                    globalReset = t;
               }


           }

           if (cmd.mid(6,6)  == "Always")
           {
               actionText  = "select 1";              // this only serves as a placeholder always returning true, the reset is used to create a seperate behaviour
           }


           if (cmd.mid(13,15) == "following days:")
           {               

              // the mid string contains list of days e.g. Mon,Tue,Fri
              // sql checkes if the current day is in this list of days

              actionText  = "SELECT 1 from DUAL where (select FIND_IN_SET((SELECT left(DAYNAME(DATE(fnGetSchedulerDateTime())),3)),'" + cmd.mid(28) + "'))";
           }

           if (cmd.mid(6,16) == "Repeat every day")
           {

              // sames as above just substituing a set if days of the week for all days of the week

              actionText  = "SELECT 1 from DUAL where (select FIND_IN_SET((SELECT left(DAYNAME(DATE(fnGetSchedulerDateTime())),3)),'Mon,Tue,Wed,Thu,Fri,Sat,Sun'))";
           }

           if (cmd.mid(6,13) == "Repeat every:")
           {
               QString q;
               q=cmd.mid(18).section(':',1,1);
               q=q.section(" days",0,0);

               QSqlQuery sched;
               sched.clear();

               sched.prepare("SELECT DATE(fnGetSchedulerDateTime())");

               sched.exec();

               sched.next();

               QString dateString;
               dateString = sched.value(0).toString();


               // use the day repeat (q) as a modulus on the diff between today and the current date (when the sql is executed)

               actionText  = "SELECT 1 from DUAL Where (SELECT DATEDIFF('" + dateString + "',DATE(fnGetSchedulerDateTime())) %" + q + " = 0)";


           }

           if (cmd.mid(6,19) == "Repeat starting at:")
           {
               QString t;
               t=cmd.mid(26,5) + ":00";

               QString q;
               q=cmd.mid(37).section(':',1,1);
               q=q.section(" hours",0,0);

               // use the hours repeat (q) as a modulus on the diff between today and the current date in hours (when the sql is executed)

               actionText = "SELECT 1 from DUAL WHERE (SELECT TIMESTAMPDIFF(HOUR,TIMESTAMP(DATE(fnGetSchedulerDateTime()),'" + t + "'),fnGetSchedulerDateTime()) % " + q + " = 0)";


           }

           if (cmd.mid(6,9) == "Remind me")
           {
               QString q;
               q=cmd.section("after",1,1);
               q=q.section(" seconds",0,0);

               ui->resetSpinBox->setValue(q.toInt());

           }

           addActionRulesRow(sequenceName,ruletext,actionText,ruleCount,"R");

           ruleCount++;

       }
    }

    // Now add the reset condition

    QString sId;

    sId = createResetCondition(sequenceName);  // create condition e.g. 708 answerDoorBell

    if (sId == "")
    {
        qDebug()<<"Problem creating reset condition!";
        return;
    }

    // add the rule cehcking the condition

    addActionRulesRow(sequenceName,"Auto:" + sId + sequenceName + " is true", "SELECT * FROM Sensors WHERE sensorId = " + sId + " AND value = 'true'" ,ruleCount,"R");

    ruleCount++;

    // create the reset procedure

    QString reset;
//    reset.setNum(ui->resetSpinBox->value());

    // always default reset to 1 minute

    if (ui->alwaysCheckBox->isChecked())   // this implies that user turned off the reminder bit
    {
        // we reset at midnight
        addActionRulesRow("reset-" + sequenceName,sId + sequenceName + " is false", "SELECT * FROM Sensors WHERE sensorId = " + sId + " AND value = 'false'",0,"R");
        addActionRulesRow("reset-" + sequenceName,sId + sequenceName + " And it is between 1am and 2am", "CALL spBetweenTimeCheck('01:00:00','02:00:00') " ,1,"R");
     }
    else
    {                                    // we reset/remind in this bit
       reset.setNum(globalReset);
       qDebug()<<"Reset: "<< reset;
       addActionRulesRow("reset-" + sequenceName,sId + sequenceName + " is false and has been for " + reset + " seconds", \
                                   "SELECT * FROM Sensors WHERE sensorId = " + sId + " AND value = 'false' and lastUpdate+INTERVAL " + reset + " SECOND <= NOW()" ,0,"R");
    }



    addActionRulesRow("reset-" + sequenceName,"SET ::" + sId + "::" + sequenceName + " TO true", "cond,0," + sId + ",true" ,2,"A");

    addSequenceRow   ("reset-" + sequenceName, "Reset condition " + sequenceName, 1, 80, 1);




    for (int i = 0; i < learnedList.size(); ++i)
    {
        if (!learnedList.at(i).contains("When:"))
        {
            qDebug()<< "Leaerned list actions:: " << learnedList.at(i);

          // process for sql

          QString cmd = learnedList.at(i);
          QString ruletext;
          QString actionText;
          QString command;

          ruletext = cmd;

          if (cmd.left(3) == "say")
          {
          //    ruletext = defaultRobot + " says '" + cmd.mid(4).remove('"') + "' and wait for completion";
              actionText = "speak," + defaultRobot.section("::",1,1) + "," + cmd.mid(4).remove('"') + ",,wait";
          }
          else
          {
            // get the bit in the brackets

        //    qDebug()<< cmd.section("(",1,1);



            command = cmd.section("(",1,1);
            command = command.section(")",0,0);

            // does his already exist?

            qDebug()<<"learned list      :: " <<command;

            query.clear();

            query.prepare("SELECT * FROM Sequences where name = '" + command + "'" + " AND experimentalLocationId = '" +  expLocation + "'");

            query.exec();

            if (query.size() > 0)         // found
            {
            //    ruletext = cmd;
             //   ruletext = "Execute sequence '" + command + "' on " + defaultRobot;
                actionText = "sequence," + defaultRobot.section("::",1,1) + "," + command;
            }
            else
            {
              if (command.left(4) == "goto")    // need to create entirely new sequence
              {
                   createGOTOsequence(command);
              //     ruletext = cmd;
               //    ruletext = "Execute sequence '" + command + "' on " + defaultRobot;
                   actionText = "sequence," + defaultRobot.section("::",1,1) + "," + command;

              }
              else
              {
                break;
              }
            }
          }



          addActionRulesRow(sequenceName,ruletext,actionText,ruleCount,"A");

          ruleCount++;
        }
      }

    // turn the reset condition off

    addActionRulesRow(sequenceName,"Auto: SET ::" + sId + "::" + sequenceName + " TO false", "cond,0," + sId + ",false" ,ruleCount,"A");

    ruleCount++;

    // Add a row to the sequences table

    addSequenceRow(ui->behNameComboBox->currentText(), ui->behNameLineEdit->text(),ruleCount++,30,1);

    ui->allDonePushButton->show();

}

QString MainWindow::createResetCondition(QString seq)
{

    QSqlQuery query;

    query.prepare("SELECT MAX(sensorId) FROM Sensors where sensorId >699 and sensorId < 900 ");  // between 700-899

    if (!query.exec())
    {
        QMessageBox msgBox;
        msgBox.setIcon(QMessageBox::Critical);

        msgBox.setText("Can't add/update Sequence table - duplicate?");
        msgBox.exec();

        qCritical("Cannot add/update: %s (%s)",
                  db.lastError().text().toLatin1().data(),
                  qt_error_string().toLocal8Bit().data());
        return "";
    }

    int sId;

    while(query.next())
    {

        if (query.value(0).toInt() == 0)
        {
            sId = 700;
        }
        else
        {
           sId = query.value(0).toInt() + 1;
        }

   //    qDebug()<<sId<<" "<<seq;

       query.prepare("INSERT INTO Sensors VALUES (:sensorId, 'true', '0', :name, '5', 'Predicate', 'N/A', '6',NOW(),NOW(),0,'false','false',0,0,0,0)");

       query.bindValue(":sensorId",sId);
       query.bindValue(":name",seq);


       if (!query.exec())
       {

        QMessageBox msgBox;
        msgBox.setIcon(QMessageBox::Critical);

        msgBox.setText("Can't add to Sensors table - duplicate?");
        msgBox.exec();

        qCritical("Cannot add/update: %s (%s)",
                  db.lastError().text().toLatin1().data(),
                  qt_error_string().toLocal8Bit().data());
        return "";
       }
   }

   QString sI;

   return sI.setNum(sId);

}



void MainWindow::createGOTOsequence(QString seq)
{

    // a goto sequence contains not only the robot move, but also an attempt to lower the tray and flash the lights

     addActionRulesRow(seq, "Turn light on " + defaultRobot + " to white", "light," + defaultRobot.section("::",1,1) +",white", 0, "A");
     addActionRulesRow(seq, "Execute sequence 'lowerTray' on " + defaultRobot, "sequence," + defaultRobot.section("::",1,1) + ",lowerTray", 1, "A");

     QSqlQuery query;

     QString locn = seq.section("::",1,1);

 //    qDebug()<<"locn"<<locn;

     query.prepare("SELECT * from Locations WHERE locationId = " + locn);
     query.exec();

     query.next();

     QString txt = "move " + defaultRobot + " to " + seq.section("goto",1,1) + " and wait for completion";
     QString act = "base," + defaultRobot.section("::",1,1) + ",[" + query.value(3).toString() + ":" + query.value(4).toString() + ":" + query.value(7).toString() + "]," + locn + ",wait";
 //    qDebug()<<act;
     addActionRulesRow(seq, txt, act, 2, "A");

     addActionRulesRow(seq, "Turn light on " + defaultRobot + " to yellow", "light," + defaultRobot.section("::",1,1) +",yellow", 3, "A");
     addSequenceRow(seq, "Send the robot to the " + seq.section("goto",1,1) + ", lowering tray if possible (" + seq + ")" ,4,30,0);
}


void MainWindow::addSequenceRow(QString sequenceName, QString scenText, int ruleCount, int priority, int scheduled)
{
    // Add a row to the sequences table
    qDebug()<<"Sequence added:: " << sequenceName<<" "<< scenText;
    QSqlQuery query;

    query.prepare("INSERT INTO Sequences VALUES (:name, :priority, :inter, :ruleCount, :actionCount, :sched, :exec, :scen, :scendesc, :expid)");

    query.bindValue(":name",sequenceName);
    query.bindValue(":priority",priority);                                 // default for user generated behaviours
    query.bindValue(":inter",1);                                     // always interuptable
    query.bindValue(":sched",scheduled);                                     // wkether schedulable
    query.bindValue(":ruleCount",ruleCount);
    query.bindValue(":actionCount",0);
    query.bindValue(":exec",0);
    query.bindValue(":scen","User Generated");
    query.bindValue(":scendesc",scenText);
    query.bindValue(":expid",experimentLocation);

    if (!query.exec())
    {

        qDebug() << query.lastQuery();

        QMessageBox msgBox;
        msgBox.setIcon(QMessageBox::Critical);

        msgBox.setText("Can't add/update Sequence table - duplicate?");
        msgBox.exec();

        qCritical("Cannot add/update: %s (%s)",
                  db.lastError().text().toLatin1().data(),
                  qt_error_string().toLocal8Bit().data());
        return;

    }


}



void MainWindow::addActionRulesRow(QString sequenceName, QString ruletext, QString actionText, int ruleCount, QString ruleActionFlag)
{
    qDebug()<<"ActionRule added:: " << sequenceName<<" "<< ruletext << " " << actionText;

    QSqlQuery query;

    query.clear();
    query.prepare("INSERT INTO ActionRules VALUES (:name, :ruleOrder, :ruleType, :notConnector, :andOrConnector, :ruleActionText, :rule, :action, :expid)");

 //       query.prepare("INSERT INTO ActionRules VALUES ('joe', 0, 'A', 0, 'aaa', 'asdasd', 'asdasd', '', 3)");


    query.bindValue(":name",sequenceName );
    query.bindValue(":ruleOrder",ruleCount);
    query.bindValue(":ruleType",ruleActionFlag);
    query.bindValue(":andOrConnector",0);     // default to no connector
    query.bindValue(":notConnector",0);
    query.bindValue(":ruleActionText",ruletext);
    query.bindValue(":expid",experimentLocation);

    if (ruleActionFlag == "R")
    {
        query.bindValue(":rule",actionText);
        query.bindValue(":action","");
    }
    else
    {
        query.bindValue(":rule","");
        query.bindValue(":action",actionText);
    }

    if (!query.exec())
    {
        qDebug() << query.lastQuery();

        QMessageBox msgBox;
        msgBox.setIcon(QMessageBox::Critical);

 //       msgBox.setText("Can't add/update actionRules table - duplicate?");

        QString r;
        r.setNum(ruleCount);
        r = r + " " + sequenceName + " " + expLocation + " " + ruletext + " " + actionText;
        msgBox.setText(r);
        msgBox.exec();

        qDebug()<<query.lastError();
        qDebug()<<query.lastError().databaseText();
                qDebug()<<query.lastError().text();

     //   qCritical("Cannot add/update: %s (%s)",
     //             db..lastError().text().toLatin1().data(),
     //             qt_error_string().toLocal8Bit().data());
        return;
    }


}

void MainWindow::on_deletePushButton_clicked()
{

    deleteExistingBehaviour(ui->behNameComboBox->currentText());

    QMessageBox msgBox;
    msgBox.setText(ui->behNameComboBox->currentText() + " has been forgotten!");
    msgBox.exec();

    initialiseGUI();


}

void MainWindow::deleteExistingBehaviour(QString name)
{

    QSqlQuery query;

    query.prepare("DELETE FROM Sequences WHERE name = :name or name = :resetname AND  experimentalLocationId = :expid");

    query.bindValue(":name",name);
    query.bindValue(":resetname","reset-" + name);
    query.bindValue(":expid",expLocation);

    if (!query.exec())
    {
        QMessageBox msgBox;
        msgBox.setIcon(QMessageBox::Critical);

        msgBox.setText("Database error - can't delete from Sequence table!");
        msgBox.exec();

        qCritical("Cannot delete: %s (%s)",
                  db.lastError().text().toLatin1().data(),
                  qt_error_string().toLocal8Bit().data());
        return;

    }

    // delete all instances in the actionrules table

    query.clear();

    query.prepare("DELETE FROM ActionRules WHERE name = :name or name = :resetname AND  experimentalLocationId = :expid");

    query.bindValue(":name",name);
    query.bindValue(":resetname","reset-" + name);
    query.bindValue(":expid",expLocation);

    if (!query.exec())
    {
        QMessageBox msgBox;
        msgBox.setIcon(QMessageBox::Critical);

        msgBox.setText("Database error - can't delete from ActionRules table!");
        msgBox.exec();

        qCritical("Cannot delete: %s (%s)",
                  db.lastError().text().toLatin1().data(),
                  qt_error_string().toLocal8Bit().data());
        return;

    }

    // remove the reset condition

    query.clear();

    query.prepare("DELETE FROM Sensors WHERE name = :name and sensorId > 699 and sensorId < 900");

    query.bindValue("name",name);

    if (!query.exec())
    {
        QMessageBox msgBox;
        msgBox.setIcon(QMessageBox::Critical);

        msgBox.setText("Database error - can't delete from Sensors table!");
        msgBox.exec();

        qCritical("Cannot delete: %s (%s)",
                  db.lastError().text().toLatin1().data(),
                  qt_error_string().toLocal8Bit().data());
        return;

    }

}

void MainWindow::on_allDonePushButton_clicked()
{
        initialiseGUI();


}


void MainWindow::on_alwaysCheckBox_clicked(bool checked)
{
    if (checked)
    {

        ui->resetSpinBox->setValue(0);
    }
    else
    {

        ui->resetSpinBox->setValue(60);

    }

}

void MainWindow::on_sittingSofaCheckBox_clicked(bool checked)
{
    if (checked)
    {
      ui->happeningSpinBox->setEnabled(true);
      ui->hasHappendedSpinBox->setEnabled(true);
    }
}

void MainWindow::on_TVOncheckBox_clicked(bool checked)
{
    if (checked)
    {
      ui->happeningSpinBox->setEnabled(true);
      ui->hasHappendedSpinBox->setEnabled(true);
    }
}

void MainWindow::on_happeningSpinBox_valueChanged(int val)
{
    if (val > 0)
    {
      ui->hasHappendedSpinBox->setValue(0);
    }
}

void MainWindow::on_hasHappendedSpinBox_valueChanged(int val)
{
    if (val > 0)
    {
      ui->happeningSpinBox->setValue(0);
    }
}

void MainWindow::on_kitechCupboadsCheckBox_clicked(bool checked)
{
    if (checked)
    {
      ui->happeningSpinBox->setEnabled(true);
      ui->hasHappendedSpinBox->setEnabled(true);
    }
}

void MainWindow::on_kitchenTapsCheckBox_clicked(bool checked)
{
    if (checked)
    {
      ui->happeningSpinBox->setEnabled(true);
      ui->hasHappendedSpinBox->setEnabled(true);
    }
}

void MainWindow::on_sittingTableCheckBox_clicked(bool checked)
{
    if (checked)
    {
      ui->happeningSpinBox->setEnabled(true);
      ui->hasHappendedSpinBox->setEnabled(true);
    }
}

void MainWindow::on_bigCupboardCheckBox_clicked(bool checked)
{
    if (checked)
    {
      ui->happeningSpinBox->setEnabled(true);
      ui->hasHappendedSpinBox->setEnabled(true);
    }
}

void MainWindow::on_smallCupboardCheckBox_clicked(bool checked)
{
    if (checked)
    {
      ui->happeningSpinBox->setEnabled(true);
      ui->hasHappendedSpinBox->setEnabled(true);
    }
}

void MainWindow::on_microwaveCheckBox_clicked(bool checked)
{
    if (checked)
    {
      ui->happeningSpinBox->setEnabled(true);
      ui->hasHappendedSpinBox->setEnabled(true);
    }
}

void MainWindow::on_fridgeCheckBox_clicked(bool checked)
{
    if (checked)
    {
      ui->happeningSpinBox->setEnabled(true);
      ui->hasHappendedSpinBox->setEnabled(true);
    }
}

void MainWindow::on_doorbellCheckBox_clicked(bool checked)
{
    if (checked)
    {
//      ui->happeningSpinBox->setEnabled(true);
//      ui->hasHappendedSpinBox->setEnabled(true);
    }
}

void MainWindow::on_bathTapsCheckBox_clicked(bool checked)
{
    if (checked)
    {
      ui->happeningSpinBox->setEnabled(true);
      ui->hasHappendedSpinBox->setEnabled(true);
    }
}

void MainWindow::generateForWithinSQL(QString command, QString cmd, QString sql, QString &actionText )
{
    QString forStr = cmd + "For";
    QString withinStr = cmd + "Within";


    qDebug()<<command;
    qDebug()<<cmd;
    qDebug()<<forStr;
    qDebug()<<withinStr;

    if (command.contains(forStr))
    {
      actionText = sql;
      QString mins = command.section(":",1,1);
      actionText += " and lastUpdate+INTERVAL " + mins + " MINUTE <= NOW()";
    }
    else
    {
      if (command.contains(withinStr))
      {
        actionText = sql;
        QString mins = command.section(":",1,1);
        actionText += " and lastUpdate+INTERVAL " + mins + " MINUTE >= NOW()";
      }
      else
      {
         if (command == cmd)
         {
            actionText = sql;
         }
      }
   }

}

QString MainWindow::createForWithinMsg(QString cmd, int happening, int happened, QString normal, QString normalHappining, QString normalHappened)
{
    if (happening > 0)
    {
        QString t;
        t.setNum(happening);
        return normalHappining + t + " mins (" + cmd + "For:" + t + ":)";
    }
    else
    {
       if (happened > 0)
       {
           QString t;
           t.setNum(happened);
           return normalHappened + t + " mins (" + cmd + "Within:" + t + ":)";
       }
       else
       {
         return normal + "(" + cmd + ")";
       }
    }

}

void MainWindow::on_everyNHoursSpinBox_valueChanged(int val )
{
    if (val > 0)
    {
       ui->everyNdaysSpinBox->setValue(0);
       ui->resetSpinBox->setValue(val * 3600);
       ui->resetSpinBox->setEnabled(false);
       ui->alwaysCheckBox->setChecked(false);
       ui->alwaysCheckBox->setEnabled(false);
   }

    if (val == 0)
    {
        ui->resetSpinBox->setValue(60);
        ui->resetSpinBox->setEnabled(true);
        ui->alwaysCheckBox->setEnabled(true);
    }

}

void MainWindow::on_bathDoorCheckBox_clicked(bool checked)
{
    if (checked)
    {
      ui->happeningSpinBox->setEnabled(true);
      ui->hasHappendedSpinBox->setEnabled(true);
    }
}

void MainWindow::on_bathFlushCheckBox_clicked(bool checked)
{
    if (checked)
    {
      ui->happeningSpinBox->setEnabled(true);
      ui->hasHappendedSpinBox->setEnabled(true);
    }
}



void MainWindow::on_repeatFinishedPushButton_clicked()
{
    fillFinalView();

    ui->alreadyKnowWidget->setTabEnabled(12,true);
    ui->alreadyKnowWidget->setCurrentIndex(12);
}

void MainWindow::on_repeatLearnItPushButton_clicked()
{
    bool addedToLearn = false;
    ui->repeatFinishedPushButton->show();

    if (ui->resetSpinBox->value() > 0)
    {
        int v;
        v = ui->resetSpinBox->value();

        int days = v / 86400;
        v = v%86400;

        int hours=v/3600;
        v = v%3600;

        int min=v/60;
        v = v%60;

        int sec = v;

        QString txt;

        if (days > 0)
        {
            QString d;
            d.setNum(days);
            txt += d + " days ";
        }

        if (hours > 0)
        {
            QString h;
            h.setNum(hours);
            txt += h + " hours ";
        }

        if (min > 0)
        {
            QString m;
            m.setNum(min);
            txt += m + " minutes ";
        }

        if (sec > 0)
        {
            QString s;
            s.setNum(sec);
            txt += s + " seconds ";
        }

        addToLearningList("When: Remind me/Reset after " +  txt);
        addedToLearn = true;
    }



    if (!addedToLearn)
    {
        QMessageBox msgBox;
        msgBox.setIcon(QMessageBox::Warning);

        msgBox.setText("You need to choose something for the robot to learn!");
        msgBox.exec();

       return;

    }




}

void MainWindow::on_changePushButton_clicked()
{

    QSqlQuery query;
    query.prepare("SELECT ruleActiontext FROM ActionRules WHERE name = :name AND experimentalLocationId = :locn");
    query.bindValue(":name",ui->behNameComboBox->currentText());
    query.bindValue(":locn", expLocation);

    query.exec();

    while (query.next())
    {
        if (!query.value(0).toString().contains("Auto:"))
        {
          ui->learnedSoFarListWidget->addItem(query.value(0).toString());
        }
    }

    on_startLearningPushButton_clicked();
 //   fillFinalView();

}


