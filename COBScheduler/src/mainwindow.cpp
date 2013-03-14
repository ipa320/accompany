

#include "../include/COBScheduler/mainwindow.h"
#include "ui_mainwindow.h"

#include <QSqlRelationalTableModel>
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
#include <QThread>
#include <QTimer>

#define NO_PROBLEMS 0
#define ACTIONRULES_DB_ERROR_SELECT 1
#define ACTIONRULES_DB_ERROR_UPDATE 2
#define USER_TIMEOUT 3
#define USERINTGUI_DB_ERROR_SELECT 4
#define RULES_INVALID 5
#define ACTIONGOALS_DB_ERROR_UPDATE 6
#define SCRIPTSERVER_EXECUTION_FAILURE 7

bool terminationAllowed;

class schedulerThread : public QThread {
    protected:
        void run();

    };

QSqlDatabase db0,db1, db2, db3, db4, db5, db6, db7;     // a number of seprate connections to avoid conflicts
bool dbOpen;

//QSqlQueryModel* sequences;
//QSqlQueryModel* actionRules;

int logTableRow;

Client* client;

Ui::MainWindow *uiLink;
MainWindow *mainW;

QString currentlyExecutingSequence;
int     currentlyExecutingPriority;
QString currentlyExecutingCanInterrupt;
int     currentlyExecutingRow;

int executionResult;

// for script server

QString fname;
QString cname;
QString pname;
QString pname1;
QString wait;
QString dname;




 void schedulerThread::run()
 {
    qDebug()<<"Thread starting " << currentlyExecutingSequence;

    executionResult = mainW->executeSequence(currentlyExecutingSequence, false);

    qDebug()<<"Thread ending " << currentlyExecutingSequence;

    uiLink->sequenceTableWidget->item(currentlyExecutingRow,1)->setBackgroundColor(Qt::white);

    currentlyExecutingSequence = "";
    currentlyExecutingPriority = -1;
    currentlyExecutingCanInterrupt = "Yes";
    currentlyExecutingRow = 0;
 }


 schedulerThread* sched;


MainWindow::MainWindow(int argc, char** argv, QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    uiLink = ui;
    mainW=this;

    ui->logTableWidget->clear();
    ui->logTableWidget->verticalHeader()->setVisible(false);
    ui->logTableWidget->horizontalHeader()->setVisible(false);
    ui->logTableWidget->setColumnCount(1);

    logTableRow = -1;

    QString host, user, pw;


    closeDownRequest = false;

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


    if (host == "") host = "localhost";
    if (user == "") user = "rhUser";
    if (pw=="")     pw = "waterloo";


   ui->userlabel->setText(user + ":" + host);

   if (!openAllDatabaseConnections(host, user, pw))
   {
       closeDownRequest = true;
       return;
   }


    ui->evaluatePushButton->setEnabled(false);
    ui->evaluateAllPushButton->setEnabled(true);
    ui->executePushButton->setEnabled(false);
    ui->stopSchedulerPushButton->setEnabled(false);

    ui->showNonSchedcheckBox->setChecked(false);

    // fill with sequences

    //sequences = new QSqlQueryModel();
    //actionRules = new QSqlQueryModel();


    if (!fillSequenceTable())
    {
        closeDownRequest = true;
        return;
    }

    ros::init(argc, argv, "COBScheduler");

    logMessage("ROS initialised.");

    client=new Client("/script_server", true);

    qDebug()<<"Waiting for script_server...";

    if (!client->waitForServer(ros::Duration(30.0))) //will wait for 30 secs
    {

        QMessageBox msgBox;
        msgBox.setIcon(QMessageBox::Critical);

        msgBox.setText("ROS error - cannot connect to COB script server - use: rosnode list :to see if running!");
        msgBox.exec();
        closeDownRequest = true;
        return;
    }

    logMessage("ROS - successfully connected to COB script server");

    initialise_COB_components();
    recover_COB_components();

    sched = new schedulerThread ;

    connect(&timer, SIGNAL(timeout()), this, SLOT(doSchedulerWork()));  // this is for the arbitration loop

    connect(&schedTimer, SIGNAL(timeout()), this, SLOT(updateTime()));   // this updates the time displayes on the screen

    schedTimer.start(5000);   // every 5 seconds refersh the time shown on the screen

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
    ui->actualTime->clear();
    ui->schedulerTime->clear();

    QSqlQuery query(db0);

    query.clear();

    query.prepare("SELECT time(NOW())");

    query.exec();

    while(query.next())
    {
        ui->actualTime->setText(query.value(0).toString());
    }

    query.clear();

 //   query.prepare("CALL getSchedulerTime()");

    query.exec("CALL spGetSchedulerTime()");

    while(query.next())
    {
      ui->schedulerTime->setText(query.value(0).toString());
    }

}


bool MainWindow::fillSequenceTable()
{

    QSqlQuery query(db0);

    if (ui->showNonSchedcheckBox->isChecked())
    {
        if (!query.exec("SELECT name, priority, IF(interruptable,'Yes','No')FROM Sequences ORDER BY schedulable DESC, priority DESC, RAND()"))
        {
            QMessageBox msgBox;
            msgBox.setIcon(QMessageBox::Critical);

            msgBox.setText("DB error - cannot select fom Sequences table!");
            msgBox.exec();
            return false;
        }
    }
    else
    {
     if (!query.exec("SELECT name, priority, IF(interruptable,'Yes','No')FROM Sequences WHERE schedulable = 1 ORDER BY priority DESC, RAND()"))
     {
        QMessageBox msgBox;
        msgBox.setIcon(QMessageBox::Critical);

        msgBox.setText("DB error - cannot select fom Sequences table!");
        msgBox.exec();
        return false;
      }
    }
    ui->sequenceTableWidget->clear();
    ui->sequenceTableWidget->setColumnCount(4);
    ui->sequenceTableWidget->setRowCount(query.size());
    ui->sequenceTableWidget->verticalHeader()->setVisible(false);

    QStringList labs;
    labs << "Name" << "Priority" << "Interruptable" << "Executable";
    ui->sequenceTableWidget->setHorizontalHeaderLabels(labs);

    int row=0;


    while(query.next())
    {

       ui->sequenceTableWidget->setItem(row,0,new QTableWidgetItem(query.value(0).toString()));
       ui->sequenceTableWidget->item(row,0)->setBackgroundColor(Qt::white);

       QTableWidgetItem *item = new QTableWidgetItem();
       item->setTextAlignment(Qt::AlignCenter);
       item->setText(query.value(1).toString());
       ui->sequenceTableWidget->setItem(row,1,item);

       ui->sequenceTableWidget->setItem(row,2,new QTableWidgetItem(query.value(2).toString()));
       ui->sequenceTableWidget->setItem(row,3,new QTableWidgetItem("No"));
       ui->sequenceTableWidget->item(row,3)->setBackgroundColor(Qt::red);

       ui->sequenceTableWidget->resizeColumnsToContents();

       row++;

   }

    return true;

}



void MainWindow::on_evaluatePushButton_clicked()
{
    QString sequenceName = ui->sequenceTableWidget->item(ui->sequenceTableWidget->currentRow(),0)->text();

    bool overallresult = evaluateRules(sequenceName, true);



    QString res;

    if (overallresult)
    {
       res = " -> result is executable.";
       ui->sequenceTableWidget->setItem(ui->sequenceTableWidget->currentRow(),3,new QTableWidgetItem("Yes"));
       ui->sequenceTableWidget->item(ui->sequenceTableWidget->currentRow(),3)->setBackgroundColor(Qt::green);
    }
    else
    {
       res = " -> result is NOT executable.";
       ui->sequenceTableWidget->setItem(ui->sequenceTableWidget->currentRow(),3,new QTableWidgetItem("No"));
       ui->sequenceTableWidget->item(ui->sequenceTableWidget->currentRow(),3)->setBackgroundColor(Qt::red);
    }


    // logging...

    QString msg = "Evaluated sequence: " + sequenceName + res;

    logMessage(msg);

    ui->sequenceTableWidget->setCurrentCell(-1,-1);
    ui->evaluatePushButton->setEnabled(false);
    ui->executePushButton->setEnabled(false);
}

void MainWindow::logMessage(QString msg)
{
  logTableRow++;
  ui->logTableWidget->setRowCount(logTableRow+1);
  ui->logTableWidget->setItem(logTableRow,0,new QTableWidgetItem(msg));
  ui->logTableWidget->resizeColumnsToContents();
  ui->logTableWidget->scrollToBottom();
}




void MainWindow::on_sequenceTableWidget_cellDoubleClicked(int row, int column)
{

   on_sequenceTableWidget_cellClicked(row, column);

}

void MainWindow::on_sequenceTableWidget_cellClicked(int row, int column)
{
    QString sequenceName = ui->sequenceTableWidget->item(row,0)->text();

    QSqlQuery query(db2);

    if (!query.exec("SELECT \
                         CASE ruleType WHEN \"R\" THEN \"Rule\" WHEN \"A\" THEN \"    Action\" END,\
                         ruleOrder, \
                         ruleActionText,\
                         CASE andOrConnector WHEN \"0\" THEN \"*\" WHEN \"2\" THEN \"Or\" WHEN \"1\" THEN \"And\" END,\
                         CASE ruleType WHEN \"R\" THEN \"False\" WHEN \"A\" THEN \" \" END\
                         FROM ActionRules WHERE name = \"" + sequenceName + "\" ORDER BY ruleOrder"))
    {

        QMessageBox msgBox;
        msgBox.setIcon(QMessageBox::Critical);

        msgBox.setText("DB error - cannot select from ActionRules table!");
        msgBox.exec();

        QApplication::quit();

    }






    ui->SQLtableWidget->clear();
    ui->SQLtableWidget->verticalHeader()->setVisible(false);

    ui->SQLtableWidget->setColumnCount(6);
    ui->SQLtableWidget->setRowCount(query.size());



    QStringList labs;
    labs << "Behaviour" << "Rule/Action" << "Rule Order" << "Description" << "And/Or" << "True/False";
    ui->SQLtableWidget->setHorizontalHeaderLabels(labs);


    int nrow=0;
    while(query.next())
    {

       ui->SQLtableWidget->setItem(nrow,0,new QTableWidgetItem(sequenceName));

       ui->SQLtableWidget->setItem(nrow,1,new QTableWidgetItem(query.value(0).toString()));

       QTableWidgetItem *item = new QTableWidgetItem();
       item->setTextAlignment(Qt::AlignCenter);
       item->setText(query.value(1).toString());
       ui->SQLtableWidget->setItem(nrow,2,item);

       ui->SQLtableWidget->setItem(nrow,3,new QTableWidgetItem(query.value(2).toString()));
       ui->SQLtableWidget->setItem(nrow,4,new QTableWidgetItem(query.value(3).toString()));
  //    qDebug()<<query.value(0).toString();
       if (query.value(0).toString() == "Rule")
       {
              ui->SQLtableWidget->setItem(nrow,4,new QTableWidgetItem(query.value(3).toString()));
              ui->SQLtableWidget->item(nrow,1)->setBackgroundColor(Qt::gray);

       }    
       else
       {
              ui->SQLtableWidget->setItem(nrow,4,new QTableWidgetItem(""));
              ui->SQLtableWidget->item(nrow,1)->setBackgroundColor(Qt::lightGray);
       }    

       ui->SQLtableWidget->setItem(nrow,5,new QTableWidgetItem(query.value(4).toString()));
   //         qDebug()<<query.value(4).toString();
       if (query.value(4).toString() == "False")
       {
          ui->SQLtableWidget->item(nrow,5)->setBackgroundColor(Qt::red);
       }
       ui->SQLtableWidget->resizeColumnsToContents();

       nrow++;
    }

    ui->evaluatePushButton->setEnabled(true);
    ui->executePushButton->setEnabled(true);
}

void MainWindow::on_evaluateAllPushButton_clicked()
{
    ui->evaluatePushButton->setEnabled(false);
    ui->executePushButton->setEnabled(false);

    int noRows = ui->sequenceTableWidget->rowCount();


    for (int i=0; i < noRows; i++)
    {
       QString sequenceName = ui->sequenceTableWidget->item(i,0)->text();

       QString res;

       bool overallresult = evaluateRules(sequenceName, false);

       if (overallresult)
       {
          ui->sequenceTableWidget->setItem(i,3,new QTableWidgetItem("Yes"));
          ui->sequenceTableWidget->item(i,3)->setBackgroundColor(Qt::green);
          res = " -> result is executable.";
       }
       else
       {
          ui->sequenceTableWidget->setItem(i,3,new QTableWidgetItem("No"));
          ui->sequenceTableWidget->item(i,3)->setBackgroundColor(Qt::red);
          res = " -> result is NOT executable.";
       }


       // logging...

       QString msg = "Evaluated sequence: " + sequenceName + res;

       logMessage(msg);

       ui->sequenceTableWidget->setCurrentCell(-1,-1);

    }

}

bool MainWindow::evaluateRules(QString sequenceName, bool display)
{


   QSqlQuery query(db3);
   QSqlQuery queryLogic(db4);

   query.clear();

//   qDebug()<< "evaluate rule sequence name: " << sequenceName;

   query.prepare("SELECT * FROM  ActionRules WHERE ruleType = 'R' AND name = :name ORDER BY ruleOrder");

   query.bindValue(":name",sequenceName );

   if (!query.exec())
   {
      qDebug() << "Can't select from actionrules table!" << query.executedQuery();
      qDebug()<< "evaluate rule sequence name: " << sequenceName;
      return false;
   }

   if (query.size() == 0)   // no rules so always true!
   {
      return true;
   }

   QString prevANDORSwitch = "undefined";
   QString ANDORSwitch;

   int recCount=0;
   bool overallresult, result;

   overallresult = result = false;

   bool finalResult = true;

   while(query.next())
  {
     if (query.value(2).toString() == "R")
     {
        recCount++;

  //      qDebug()<<query.value(4).toString()<<"  "<<query.value(6).toString();

        QString ANDORSwitch = "undefined";

        if (query.value(4).toInt() == 0)
        {
           ANDORSwitch = "AND";
        }

        if (query.value(4).toInt() == 1)
        {
           ANDORSwitch = "AND";
        }

        if (query.value(4).toInt() == 2)
        {
           ANDORSwitch = "OR";
        }

//        qDebug()<<ANDORSwitch;

        queryLogic.clear();

   //     queryLogic.prepare(query.value(6).toString());

        if (!queryLogic.exec(query.value(6).toString()))
        {
           qDebug() << "Can't execute rule from actionrules table!" << queryLogic.executedQuery();
           qDebug() << "Rule is: " << query.value(6).toString();
           return false;
        }
        int row = recCount -1;

        if(queryLogic.size() > 0)
        {
           if (display)
           {
              ui->SQLtableWidget->setItem(row,5,new QTableWidgetItem("True"));
              ui->SQLtableWidget->item(row,5)->setBackgroundColor(Qt::green);
           }
           result = true;
  //         qDebug()<<" <-- true";
       }
       else
       {
          if (display)
          {
            ui->SQLtableWidget->setItem(row,5,new QTableWidgetItem("False"));
            ui->SQLtableWidget->item(row,5)->setBackgroundColor(Qt::red);
          }
          result = false;
   //       qDebug()<<" <-- false";
       }

       if (recCount == 1)             // first record
       {
          overallresult = result;
          prevANDORSwitch = ANDORSwitch;

          if (ANDORSwitch == "Undefined" && result == false)
          {
              finalResult = false;
          }

       }
       else
       {
         if (prevANDORSwitch == "OR")
         {
             overallresult = overallresult || result;
         }

         if (prevANDORSwitch == "AND")
         {
             overallresult = overallresult && result;
         }

         if (prevANDORSwitch == "undefined")
         {
             if (ANDORSwitch == "Undefined" && result == false)
             {
                 finalResult = false;
             }
            // overallresult = overallresult && result;
         }
         prevANDORSwitch = ANDORSwitch;

     }
   }
 }

 if (finalResult)
 {
  return overallresult;
 }
 else
 {
   return finalResult;
 }

}


void MainWindow::on_executePushButton_clicked()
{
    QString sequenceName = ui->sequenceTableWidget->item(ui->sequenceTableWidget->currentRow(),0)->text();

/*
    Client client("/script_server", true);

    if (!client.waitForServer(ros::Duration(30.0))) //will wait for 30 secs
    {
        QMessageBox msgBox;
        msgBox.setIcon(QMessageBox::Critical);

        msgBox.setText("ROS error - cannot connect to script server - use: rosnode list :to see if running!");
        msgBox.exec();
        return;
    }
*/
    executionResult = executeSequence(sequenceName, true);

    checkExecutionResult();



}

int MainWindow::executeSequence(QString sequenceName, bool display)
{
    int returnResult = 0;

    bool overallresult = evaluateRules(sequenceName, display);

    qDebug()<<"Evaluated " << sequenceName << " result " << overallresult;

    if (!overallresult)       // no logner executable!
    {
 //       returnResult = RULES_INVALID;
        return returnResult;
    }

    // get the set of actions for this sequence

    QSqlQuery query(db1);

    query.clear();

    query.prepare("SELECT action FROM  ActionRules WHERE name = :name AND ruleType = \"A\" ORDER BY ruleOrder");

    query.bindValue(":name",sequenceName );

    if (!query.exec())
    {
       returnResult = ACTIONRULES_DB_ERROR_SELECT;
       qDebug()<<"Sequence Name: " << sequenceName;
       return returnResult;
    }

    // for each action - execute it via the script server

    while(query.next())
    {

      // unpack query

      QString str = query.value(0).toString();

      qDebug() << str;

      cname = str.section(',', 0, 0);
      pname = str.section(',', 2, 2);
      pname1 = str.section(',', 3, 3);
      wait  = str.section(',', 4, 4);



      // check each command in turn and form the appropriate script server message
      //
      //  function name
      //  component name
      //  parameter name
      //  duration

      // -----------
      // make the robot say something - format:  say "" ['hello'] ""
      // -----------


      if (cname == "speak")
      {
        cname = "say";
        fname = cname;
        pname = "['" + pname + "']";
        cname = "";

        returnResult = sendScriptServerMsg();

      }

      // -----------
      // set the light colour - format:   "" set_light red ""
      // -----------

      if (cname == "light"   )
      {

          cname = "set_light";
          fname = cname;
          cname = "";

          returnResult = sendScriptServerMsg();

      }

      // -----------
      // make the robot go to sleep - format: sleep "" "" 10
      // -----------

      if ( cname == "sleep" )
      {
          fname = cname;
          cname="";
          dname = pname;
          pname = "";

          returnResult = sendScriptServerMsg();
      }

      // -----------
      // actuation - format: move base [x,y,theta] ""
      //                     move tray up ""
      //                     move torso nod ""
      //                     move eyes forward ""
      //                     move head left ""
      // -----------

      if (cname == "tray" || cname == "base" || cname == "torso" || cname == "eyes" || cname == "head" )
      {
          fname = "move";

          if (cname == "base")    // base,0,[3.88:1.17:23],14,wait
          {
              qDebug()<<pname << " " << pname1;
              pname.remove('[');
              pname.remove(']');

       //       qDebug()<<pname.section(":",2,2);
              double degrees    = pname.section(":",2,2).toDouble();  // we hold on the database in degrees
       //       qDebug()<<degrees;

              double radians = degrees / 180.0 * 3.142;         // the command must be in radians
              QString rd;
              rd.setNum(radians);

              // ste the query from the rule

              pname = "[" + pname.section(":",0,0) + "," + pname.section(":",1,1) + "," + rd + "]";

              // but try to get real location if available

              if (pname1 != "")
              {
                QSqlQuery locationQuery(db7);

                locationQuery.clear();

                locationQuery.prepare("SELECT xCoord,yCoord,orientation from Locations WHERE locationId = :locn");

                locationQuery.bindValue(":locn",pname1);


                if (locationQuery.exec())
                {
                   while(locationQuery.next())
                   {
                      if (pname1 == "999")   // 999 is current user location, values set by context planner, therefore use its orientation
                      {
                          double rads = locationQuery.value(2).toFloat() / 180.0 * 3.142;
                          QString rad;
                          rad.setNum(rads);
                          pname = "[" + locationQuery.value(0).toString() + "," + locationQuery.value(1).toString() + "," + rad + "]";

                      }
                      else     // use th euser specified orientation
                      {
                         pname = "[" + locationQuery.value(0).toString() + "," + locationQuery.value(1).toString() + "," + rd + "]";
                      }
                  }
                }
                else
               {
                  qDebug()<<"Location query failed - using existing rule";
               }
            }

              qDebug()<<"Moving to:" << pname << "orient degrees: " << degrees;
          }

          returnResult = sendScriptServerMsg();

      }

      // -----------
      // The user GUI - doesn't use script server
      // -----------

      if (cname == "GUI")       // set the wait flag in db and await response
      {


          QSqlQuery GUIquery(db5);

          GUIquery.clear();

          GUIquery.prepare("UPDATE userInterfaceGUI SET guiMsgResult = NULL WHERE name = :name ");

          GUIquery.bindValue(":name",sequenceName );

          if (!GUIquery.exec())
          {
             returnResult = ACTIONRULES_DB_ERROR_UPDATE;
             return returnResult;
          }


         bool  awaitingUserResponse = true;
         int   numSeconds = 0;

         while (awaitingUserResponse)
         {

           numSeconds++;

           if (numSeconds > 60)          // what should we do if the user doesn't respond?
           {
    //          returnResult = USER_TIMEOUT;
    //          return returnResult;
               qDebug() << "User hasn't responsed to GUI...";
           }

           sleep(1);

    //       qDebug()<<numSeconds;

           GUIquery.clear();

           GUIquery.prepare("SELECT guiMsgResult FROM userInterfaceGUI WHERE name = :name ");

           GUIquery.bindValue(":name",sequenceName );

           if (!GUIquery.exec())
           {
               returnResult = USERINTGUI_DB_ERROR_SELECT;
               return returnResult;
           }

           GUIquery.next();

           QString resultFromUser = GUIquery.value(0).toString();

      //     qDebug()<<resultFromUser;
           if (resultFromUser == "0" || resultFromUser == "" ) // no response
               continue;

           awaitingUserResponse = false;       // got a response


       //    qDebug()<<pname;
       //    qDebug()<<pname.section("@",resultFromUser.toInt()-1,resultFromUser.toInt()-1);

           returnResult = mainW->executeSequence(pname.section("@",resultFromUser.toInt()-1,resultFromUser.toInt()-1), false);
        }

      }

      // -----------
      // set logical goals and conditions - doesn't use script server
      // -----------

      if (cname == "cond")
      {
          QSqlQuery Goalquery(db6);

          Goalquery.clear();

          Goalquery.prepare("INSERT INTO SensorLog (timestamp,sensorId,room,channel,value,status) VALUES (NOW(),:goalId,'','',:value,:truefalse)");

          Goalquery.bindValue(":value", pname1);
          Goalquery.bindValue(":goalId",pname);

          QString TF = "true";

          if (pname1 == "0") TF = "false";

          Goalquery.bindValue(":truefalse",TF);

          qDebug()<<"Updating goals: pane1" << pname1 << " pname " << pname;

          if (!Goalquery.exec())
          {
             returnResult = ACTIONGOALS_DB_ERROR_UPDATE;
             return returnResult;
          }
       }

      // -----------
      // recursive calls to this routine
      //-----------

       if (cname == "sequence")
       {
           returnResult = mainW->executeSequence(pname, false);
       }

   }  // end of while


   qDebug()<<"Sequence: " + sequenceName + " finished";

   return returnResult;
}




int MainWindow::sendScriptServerMsg()
{
    int returnResult = 0;

    qDebug()<<"fname " << fname <<  " cname " << cname << " pname " << pname << " dname " << dname ;

    // send the script server message

    cob_script_server::ScriptGoal goal;

    goal.function_name  = fname.toStdString();
    goal.component_name = cname.toStdString();
    goal.parameter_name = pname.toStdString();
    goal.duration       = dname.toFloat();


    if (wait == "wait")
    {
       qDebug()<<"sending message and waiting...";
       goal.blocking = true;
    }
    else
    {
       qDebug()<<"sending message...";
       goal.blocking = false;
    }

    client->sendGoalAndWait(goal);

    terminationAllowed = false;

    qDebug()<<"waiting on result...";
    client->waitForResult();
    qDebug()<<"finished waiting.";

    terminationAllowed = true;

    if (client->getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
    {
       qDebug()<<"Send Goal failed - terminating...";
       returnResult = SCRIPTSERVER_EXECUTION_FAILURE;
    }

    return returnResult;
}



void MainWindow::on_COBTestPushButton_clicked()
{

    if (!fillSequenceTable())
    {
       QApplication::quit();
    }
}


void MainWindow::COB_component(QString component, QString action)
{

    QString msg = "COB - ";

    cob_script_server::ScriptGoal goal;;

    goal.function_name= action.toStdString();
    goal.component_name=component.toStdString();
    client->sendGoalAndWait(goal);
    client->waitForResult();

    // for correct English only!

    if (action == "init") action = "initialised";
    if (action == "stop") action = "stopped";
    if (action == "recover") action = "recovered";

    if (client->getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        QMessageBox msgBox;

        msg = msg + component + " could not be " + action;
        logMessage(msg);
        msgBox.setIcon(QMessageBox::Warning);
        msgBox.setText(msg);
        msgBox.exec();
    }
    else
    {
        msg = msg + component + " successfully " + action ;
        logMessage(msg);
    }
}

void MainWindow::stop_COB_Components()
{
    COB_component("tray","stop");
    COB_component("torso","stop");
    COB_component("base","stop");
    COB_component("arm","stop");
    COB_component("sdh","stop");
}


void MainWindow::on_initialiseAllPushButton_clicked()
{
  initialise_COB_components();
}

void MainWindow::on_stopAllPushButton_clicked()
{
    stop_COB_Components();

}

void MainWindow::on_recoverAllPushButton_clicked()
{
    recover_COB_components();
}

void MainWindow::recover_COB_components()
{
    COB_component("tray","recover");
    COB_component("torso","recover");
    COB_component("base","recover");
    COB_component("arm","recover");
    COB_component("sdh","recover");
}

void MainWindow::initialise_COB_components()
{
    COB_component("tray","init");
    COB_component("torso","init");
    COB_component("base","init");
    COB_component("arm","init");
    COB_component("sdh","init");
}

void MainWindow::on_startSchedulerPushButton_clicked()
{

    logMessage("Starting scheduler...");

    ui->COBTestPushButton->setEnabled(false);
    ui->showNonSchedcheckBox->setEnabled(false);
    ui->showNonSchedcheckBox->setChecked(false);

    fillSequenceTable();


    currentlyExecutingSequence = "";
    currentlyExecutingPriority = -1;
    currentlyExecutingCanInterrupt = "Yes";
    currentlyExecutingRow = 0;

    timer.start(1500);

    logMessage("Scheduler started");

    ui->startSchedulerPushButton->setEnabled(false);
    ui->stopSchedulerPushButton->setEnabled(true);
    ui->evaluateAllPushButton->setEnabled(false);
    ui->evaluatePushButton->setEnabled(false);
    ui->executePushButton->setEnabled(false);
}

void MainWindow::on_stopSchedulerPushButton_clicked()
{

    ui->COBTestPushButton->setEnabled(true);

    logMessage("Stopping scheduler...");

    timer.stop();

    stopSequence(currentlyExecutingSequence);   // stops thread

    logMessage("Scheduler stopped");

    ui->startSchedulerPushButton->setEnabled(true);
    ui->stopSchedulerPushButton->setEnabled(false);
    ui->evaluateAllPushButton->setEnabled(true);
    ui->showNonSchedcheckBox->setEnabled(true);


}




void MainWindow::doSchedulerWork()
{
    if (sched->isRunning()) qDebug()<< "Thread is ----- Running -----" ;
    if (sched->isFinished()) qDebug()<< "Thread is **** Finished ***" ;

    if (executionResult !=0)
    {
       stopSequence(currentlyExecutingSequence);
       checkExecutionResult();
       QApplication::quit();
    }


 //    logMessage("Evaluating sequence list...");

     QString readySequence = "";
     int     readyPriority = -1;
     QString readyCanInterrupt = "No";
     int     readyRow = 0;

 //    fillSequenceTable();

     int rows = ui->sequenceTableWidget->rowCount();

     bool overallresult = false;
     bool result = false;

     for (int i= 0; i< rows; i++)
     {
        QString sequenceName = ui->sequenceTableWidget->item(i,0)->text();
        int priority         = ui->sequenceTableWidget->item(i,1)->text().toInt();
        QString CanInterrupt = ui->sequenceTableWidget->item(i,2)->text();
   
        ui->sequenceTableWidget->item(i,0)->setBackgroundColor(Qt::yellow);


  //      logMessage("Evaluating: " + sequenceName);

        result = evaluateRules(sequenceName, false);

        if (result)
        {
           ui->sequenceTableWidget->setItem(i,3,new QTableWidgetItem("Yes"));
           ui->sequenceTableWidget->item(i,3)->setBackgroundColor(Qt::green);
        }
        else
        {
          ui->sequenceTableWidget->setItem(i,3,new QTableWidgetItem("No"));
          ui->sequenceTableWidget->item(i,3)->setBackgroundColor(Qt::red);
        }

        ui->sequenceTableWidget->repaint();

        if (result)
        {
          if (!overallresult)        // beacuse the list is in priority order the first valid rule is chosen
          {
            overallresult = true;
            readySequence = sequenceName;
            readyPriority  = priority;
            readyCanInterrupt = CanInterrupt;
            readyRow = i;
          }

         }
       }

       if (!overallresult) return;

        logMessage("Ready to execute: " + readySequence);
        qDebug() << "           *** Ready to run: " << readySequence;



        if (currentlyExecutingSequence == ""  )  // nothing currently executing
        {
            qDebug() << "                      *** starting to run: " << readySequence << " started running << ****";

           runSequence(readySequence,readyPriority, readyCanInterrupt, readyRow);         // run the ready sequence
        }
        else
        {
          if (readyPriority > currentlyExecutingPriority && currentlyExecutingCanInterrupt == "Yes") // ready and higher priority
          {
           qDebug() << "***!!! " << readySequence << " usurped " << currentlyExecutingSequence << " !!!****";
           logMessage(readySequence + " usurped " + currentlyExecutingSequence);

           stopSequence(currentlyExecutingSequence);

           runSequence(readySequence, readyPriority, readyCanInterrupt, readyRow);
          }
          else
          {
              if (readySequence == currentlyExecutingSequence)
              {
                 qDebug() << "      *** " << readySequence << " in progress << ****";
                 logMessage(readySequence + " in progress...");
              }
          }
         }



}
void MainWindow::stopSequence(QString sequenceName)
{
    logMessage("Stopping: " + currentlyExecutingSequence);
    ui->sequenceTableWidget->item(currentlyExecutingRow,1)->setBackgroundColor(Qt::white);

    QSqlQuery GUIquery(db7);  // kill the gui if necessary

    GUIquery.clear();

    GUIquery.prepare("UPDATE userInterfaceGUI SET guiMsgResult = 0 WHERE name = :name ");

    GUIquery.bindValue(":name",sequenceName );

    GUIquery.exec();

    while (!terminationAllowed)
    {
        qDebug()<<"................Waiting on termination..............";
        sleep(0.5);
    }

    sched->terminate();

    currentlyExecutingSequence = "";
}

void MainWindow::runSequence(QString sequenceName, int priority, QString CanInterrupt, int row)
{
    logMessage("Executing: " + sequenceName);
    ui->sequenceTableWidget->item(row,1)->setBackgroundColor(Qt::cyan);

    currentlyExecutingSequence = sequenceName;
    currentlyExecutingPriority = priority;
    currentlyExecutingCanInterrupt = CanInterrupt;
    currentlyExecutingRow = row;

    sched->start();

    // checkExecutionResult();

}

void MainWindow::checkExecutionResult()
{
   QMessageBox msgBox;

   switch (executionResult)
   {
     case NO_PROBLEMS:
       return;

     case ACTIONRULES_DB_ERROR_SELECT:
       msgBox.setIcon(QMessageBox::Critical);
       msgBox.setText("DB error - cannot select from ActionRules table!");
       msgBox.exec();
       break;

     case ACTIONRULES_DB_ERROR_UPDATE:
       msgBox.setIcon(QMessageBox::Critical);
       msgBox.setText("DB error - cannot update ActionRules table!");
       msgBox.exec();
       break;

     case USER_TIMEOUT:
      msgBox.setIcon(QMessageBox::Warning);
      msgBox.setText("user took too long to answer question - continuing!");
      msgBox.exec();
      break;

    case USERINTGUI_DB_ERROR_SELECT:
     msgBox.setIcon(QMessageBox::Critical);
     msgBox.setText("DB error - cannot select from UserInterfaceGUI table!");
     msgBox.exec();
     break;

   case RULES_INVALID:
    msgBox.setIcon(QMessageBox::Critical);
    msgBox.setText("The sequence was not executed as the rule set is not valid!");
    msgBox.exec();
    break;

   case ACTIONGOALS_DB_ERROR_UPDATE:
     msgBox.setIcon(QMessageBox::Critical);
     msgBox.setText("DB error - cannot update SensorLog table!");
     msgBox.exec();
     break;

   case SCRIPTSERVER_EXECUTION_FAILURE:
     msgBox.setIcon(QMessageBox::Critical);
     msgBox.setText("Script server failed to execute command!");
     msgBox.exec();
     break;

  }





}
void MainWindow::on_showNonSchedcheckBox_toggled(bool checked)
{
    ui->startSchedulerPushButton->setEnabled(!checked);

    fillSequenceTable();


}

bool MainWindow::openDatabase(QString dbName, QString host, QString user, QString pw, QSqlDatabase& db)
{
    db = QSqlDatabase::addDatabase("QMYSQL",dbName);

    db.setHostName(host);
    db.setDatabaseName("Accompany");
    db.setUserName(user);
    db.setPassword(pw);

    dbOpen = db.open();

    if (!dbOpen) {

        QMessageBox msgBox;
        msgBox.setIcon(QMessageBox::Critical);

        msgBox.setText("Database error - login problem - see console log! Connection: " + dbName );
        msgBox.exec();

        qCritical("Cannot open database: %s (%s)",
                  db0.lastError().text().toLatin1().data(),
                  qt_error_string().toLocal8Bit().data());

        closeDownRequest = true;

    }
    else
    {
            logMessage("Database Opened " + dbName);
            return dbOpen;
    }

    return dbOpen;

}

bool MainWindow::openAllDatabaseConnections(QString host, QString user, QString pw)
{

    if(!openDatabase("db0",host,user,pw, db0))
    {
        return false;
    }


    if (!openDatabase("db1",host,user,pw, db1))
    {
        return false;
    }

    if (!openDatabase("db2",host,user,pw, db2))
    {
        return false;
    }

    if (!openDatabase("db3",host,user,pw, db3))
    {
        return false;
    }

    if (!openDatabase("db4",host,user,pw, db4))
    {
        return false;
    }

    if (!openDatabase("db5",host,user,pw, db5))
    {
        return false;
    }

    if (!openDatabase("db6",host,user,pw, db6))
    {
        return false;
    }

    if (!openDatabase("db7",host,user,pw, db7))
    {
        return false;
    }

    return true;
}

void MainWindow::closeAllDatabaseConnections()
{
    db0.close();
    db1.close();
    db2.close();
    db3.close();
    db4.close();
    db5.close();
    db6.close();
    db7.close();

}

void MainWindow::on_testPlannerPushButton_clicked()
{

   planNavigation("7");

   planNavigation("26");

   planNavigation("31");

}

void MainWindow::planNavigation(QString destination)
{

    QSqlQuery query(db7);

    qDebug()<<">>>>>>>>> Testing planner....";

    QString fileName = "/home/joe/JSHOP2/CareOBot/problem";

    QFile file(fileName);

    if (!file.open(QIODevice::WriteOnly))
    {
        qDebug()<< "Cannot open file for writing: " << qPrintable(file.errorString());
        return;
    }

    QTextStream out(&file);

    out << "; this is an auto-generated file defining a navigation problem for the Care-O-Bot"<< endl;
    out << ";"<< endl;
    out << "; - The current state of robot and its location adjacency conditions are extracted from the Robot House database"<< endl;
    out << ";"<< endl;
    out << ""<< endl;
    out << "(defproblem problem navigation"<< endl;
    out << ""<< endl;
    out << "; we define the state of the house and the robot"<< endl;
    out << ""<< endl;
    out << "  ("<< endl;

    query.clear();
    query.prepare("SELECT L.name, R.locationId from Accompany.Robot R, Accompany.Locations L\
                                    WHERE R.locationId = L.locationId AND robotId = 0");

    if (!query.exec())
    {
       qDebug() << "Can't select from robot table!" << query.executedQuery();
       return;
    }

    while(query.next())
    {
       out << "     (at care-o-bot " << query.value(0).toString().simplified().replace(" ","") << query.value(1).toString() << "); define current location of robot"<< endl;
    }

    out << ""<< endl;
    out << "; define all the locations"<< endl;
    out << ""<< endl;


    query.clear();
    query.prepare("SELECT name, locationId FROM  Locations where ValidRobotLocation = 1");

    if (!query.exec())
    {
       qDebug() << "Can't select from Locations table!" << query.executedQuery();
       return;
    }

    while(query.next())
    {
       out << "     (room " << query.value(0).toString().simplified().replace(" ","") << query.value(1).toString() << ")"<< endl;
    }

    out << ""<< endl;

    query.clear();

    query.prepare("SELECT  L.name,  A.locationFrom,\
                  L1.name, A.locationTo, A.cost \
                              FROM LocationAdjacency A, \
                                   Locations L, \
                                   Locations L1\
                                       Where A.locationFrom = L.locationId and\
                                     A.locationTo   = L1.locationId");

    if (!query.exec())
    {
       qDebug() << "Can't select from LocationAdjacency table!" << query.executedQuery();
    }

    out << "; now all the adjacency conditions and in reverse" << endl;
    out << ""<< endl;

    while(query.next())
    {

        out << "     (adjacentTo " << query.value(0).toString().simplified().replace(" ","") << query.value(1).toString() << " "\
                                   << query.value(2).toString().simplified().replace(" ","") << query.value(3).toString() << " "\
                                   << query.value(4).toString() << ")" << endl;
        out << "     (adjacentTo " << query.value(2).toString().simplified().replace(" ","") << query.value(3).toString() << " "\
                                   << query.value(0).toString().simplified().replace(" ","") << query.value(1).toString() << " "\
                                   << query.value(4).toString() << ")" << endl;
    }


    out << ""<< endl;
    out << "  )"<< endl;
    out << ""<< endl;
    out << "; now define the goal"<< endl;
    out << ""<< endl;
    out << "  ("<< endl;
    out << ""<< endl;

    // create the goal

    query.clear();
    query.prepare("SELECT L.name, L.locationId from Locations L WHERE L.locationId = " + destination);

    if (!query.exec())
    {
       qDebug() << "Can't select from locations table!" << query.executedQuery();
       return;
    }

    while(query.next())
    {
       out << "     (navigate care-o-bot " << query.value(0).toString().simplified().replace(" ","") << query.value(1).toString() << "); define current location of robot"<< endl;
    }

    out << ""<< endl;

    out << "  )"<< endl;
    out << ")"<< endl;

    file.close();

    FILE *in;

    char buff[512];

    if(!(in = popen("/home/joe/JSHOP2/CareOBot/doPlanning.sh", "r")))
    {
      qDebug() << "Can't open output for popen call!";
      return;
    }

    while(fgets(buff, sizeof(buff), in)!=NULL)
    {
        qDebug() << buff;

    }

    pclose(in);


}

// not ready yet!!
/*
int MainWindow::doNavigation(QString destination)
{
 // base,0,[3.88:1.17:23],14,wait

    qDebug()<<pname << " " << pname1;
    pname.remove('[');
    pname.remove(']');

//       qDebug()<<pname.section(":",2,2);
    double degrees    = pname.section(":",2,2).toDouble();  // we hold on the database in degrees
//       qDebug()<<degrees;

    double radians = degrees / 180.0 * 3.142;         // the command must be in radians
    QString rd;
    rd.setNum(radians);

    // ste the query from the rule

    pname = "[" + pname.section(":",0,0) + "," + pname.section(":",1,1) + "," + rd + "]";

    // but try to get real location if available

    if (pname1 != "")
    {
      QSqlQuery locationQuery(db7);

      locationQuery.clear();

      locationQuery.prepare("SELECT xCoord,yCoord,orientation from Locations WHERE locationId = :locn");

      locationQuery.bindValue(":locn",pname1);


      if (locationQuery.exec())
      {
         while(locationQuery.next())
         {
            if (pname1 == "999")   // 999 is current user location, values set by context planner, therefore use its orientation
            {
                double rads = locationQuery.value(2).toFloat() / 180.0 * 3.142;
                QString rad;
                rad.setNum(rads);
                pname = "[" + locationQuery.value(0).toString() + "," + locationQuery.value(1).toString() + "," + rad + "]";

            }
            else     // use th euser specified orientation
            {
               pname = "[" + locationQuery.value(0).toString() + "," + locationQuery.value(1).toString() + "," + rd + "]";
            }
        }
      }
      else
     {
        qDebug()<<"Location query failed - using existing rule";
     }
  }

    qDebug()<<"Moving to:" << pname << "orient degrees: " << degrees;
}*/
