#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <exception>

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

using namespace std;

#define NO_PROBLEMS 0
#define ACTIONRULES_DB_ERROR_SELECT 1
#define ACTIONRULES_DB_ERROR_UPDATE 2
#define USER_TIMEOUT 3
#define USERINTGUI_DB_ERROR_SELECT 4
#define RULES_INVALID 5
#define ACTIONGOALS_DB_ERROR_UPDATE 6
#define SCRIPTSERVER_EXECUTION_FAILURE 7
#define SENSORS_DB_ERROR_UPDATE 8
#define BEHAVIOUR_NOT_EXECUTABLE 9


std::string modulePath = "../UHCore/Core";

class schedulerThread: public QThread
{
protected:
    void run();
};

class threadInterruptedException: public exception
{
    virtual const char* what() const throw ()
    {
        return "Thread interrupted.";
    }
};

QSqlDatabase db0, db1, db2, db3, db4, db5, db6, db7, db8, db9, db10;     // a number of separate connections to avoid conflicts

bool dbOpen;
int logTableRow;

Ui::MainWindow *uiLink;
MainWindow *mainW;

QString currentlyExecutingSequence;
int currentlyExecutingPriority;
QString currentlyExecutingCanInterrupt;
int currentlyExecutingRow;

QString displayedSequencename;

int executionResult;

// for script server

QString fname;
QString cname;
QString rname;
QString pname;
QString pname1;
QString wait;
QString dname;

bool debugEnabled;

QString houseLocation;

bool runWithROS;
QString lv;

int indent;  // controls indentation on debug display only

void schedulerThread::run()
{
    qDebug() << "          Thread starting.... " << currentlyExecutingSequence;
    mainW->stopExecution = false;
    try
    {
        executionResult = mainW->executeSequence(currentlyExecutingSequence, false);
    }
    catch (threadInterruptedException& e)
    {
    }

    qDebug() << "          Thread ending.... " << currentlyExecutingSequence;
    qDebug() << "";
//*   uiLink->sequenceTableWidget->item(currentlyExecutingRow, 1)->setBackgroundColor(Qt::white);

    currentlyExecutingSequence = "";
    currentlyExecutingPriority = -1;
    currentlyExecutingCanInterrupt = "Yes";
    currentlyExecutingRow = 0;
}

schedulerThread* sched;
Robot *robot;
ActionHistory *actionHistory;

int rc;  // iteration count
int loopSpeed; // how fast we run the app
QString scenario; // sceanrio to use

MainWindow::MainWindow(int argc, char** argv, QWidget *parent) :
    QMainWindow(parent), ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    uiLink = ui;
    mainW = this;


    QMessageBox msgBox;

    ui->logTableWidget->clear();
    ui->logTableWidget->verticalHeader()->setVisible(false);
    ui->logTableWidget->horizontalHeader()->setVisible(false);
    ui->logTableWidget->setColumnCount(1);

    runWithROS = true;

    lv = "UH";

    opterr = 0;
    int c;
    rc=0;
    loopSpeed = 300;

    ui->speedSpinBox->setValue(300);

    while ((c = getopt(argc, argv, "rl:")) != -1)
        switch (c)
        {
        case 'r':
            runWithROS = false;
            qDebug() << "Running without ROS calls";
            break;
        case 'l':
            lv = optarg;
            break;
        default:
            lv = "UH";
            break;
        }

    qDebug() << "Running as:" << lv;

    logTableRow = -1;

    QString host, user, pw, dBase;

    closeDownRequest = false;

    bool ok;

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

    ui->userlabel->setText(lv + ":" + user + ":" + host + ":" + dBase);

    if (!openAllDatabaseConnections(host, user, pw, dBase))
    {
        closeDownRequest = true;
        return;
    }

    // Get the apartment/house location

    QSqlQuery query(db0);
    query.prepare("SELECT ExperimentalLocationId FROM SessionControl where sessionId = 1");
    query.exec();

    if (query.next())
    {
        houseLocation = query.value(0).toString();
    }
    else
    {
        QMessageBox msgBox;
        msgBox.setIcon(QMessageBox::Critical);
        msgBox.setText("Cannot read session control table from database!");
        msgBox.exec();
        closeDownRequest = true;
        return;
    }

// Get the scenario
    QStringList items;

    query.clear();
    query.prepare("SELECT scenario FROM ScenarioType where scenario != 'Operator' AND scenario != 'User Generated'");
    query.exec();

    if (query.exec())
    {
       while (query.next())
      {
        items += query.value(0).toString();
      }
    }
    else
    {
        QMessageBox msgBox;
        msgBox.setIcon(QMessageBox::Critical);
        msgBox.setText("Cannot read ScenarioTypel table from database!");
        msgBox.exec();
        closeDownRequest = true;
        return;
    }


    QString item = QInputDialog::getItem(this, tr("QInputDialog::getItem()"),
                                         tr("Scenario:"), items, 0, false, &ok);


    if (ok && !item.isEmpty())
        scenario = item;

    ui->sceanrioLabel->setText(scenario);
    ui->evaluatePushButton->setEnabled(false);
    ui->evaluateAllPushButton->setEnabled(true);
    ui->executePushButton->setEnabled(false);
    ui->stopSchedulerPushButton->setEnabled(false);

    ui->showNonSchedcheckBox->setChecked(false);

    displayedSequencename = "";

    // fill with sequences

    //sequences = new QSqlQueryModel();
    //actionRules = new QSqlQueryModel();

    if (!fillSequenceTable(scenario))
    {
        closeDownRequest = true;
        return;
    }

    sched = new schedulerThread;

    connect(&timer, SIGNAL(timeout()), this, SLOT(doSchedulerWork()));  // this is for the arbitration loop
    connect(&schedTimer, SIGNAL(timeout()), this, SLOT(updateTime()));  // this updates the time displays on the screen

    schedTimer.start(5000);   // every 5 seconds refresh the time shown on the screen

    debugEnabled = false;

    ui->GUIgroupBox->setEnabled(false);

    indent = 0;

    if (runWithROS)
    {
        actionHistory = new ActionHistory(modulePath);
        // open up the python interface to the robot
        robot = new Robot(modulePath); //use the current robot specified in the sessioncontrol table
        if (robot->getName() == "")
        {
            // This appears to happen sometimes if the application has not completely loaded? before reaching this line
            // nothing we can do, so panic and exit
            QMessageBox msgBox;
            msgBox.setIcon(QMessageBox::Critical);
            msgBox.setText(
                "Robot interface did not build!  This seems to be related to clicking through the dialogs too quickly.");
            msgBox.exec();
            closeDownRequest = true;
            return;
        }
    }
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::changeEvent(QEvent *e)
{
    QMainWindow::changeEvent(e);
    switch (e->type())
    {
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

    while (query.next())
    {
        ui->actualTime->setText(query.value(0).toString());
    }

    query.clear();
    //   query.prepare("CALL getSchedulerTime()");
    query.exec("CALL spGetSchedulerTime()");

    while (query.next())
    {
        ui->schedulerTime->setText(query.value(0).toString());
    }

}

bool MainWindow::fillSequenceTable(QString scenario)
{

    QSqlQuery query(db8);

    if (ui->showNonSchedcheckBox->isChecked())
    {
        QString qry =
            "SELECT name, priority, IF(interruptable,'Yes','No')FROM Sequences Where experimentalLocationId = "\

            + houseLocation + " ORDER BY name"; //schedulable DESC, priority DESC, RAND()" ;

        if (!query.exec(qry))
        {
            QMessageBox msgBox;
            msgBox.setIcon(QMessageBox::Critical);

            msgBox.setText("DB error - (1) cannot select fom Sequences table!");
            msgBox.exec();
            return false;
        }
    }
    else
    {
        QString qry =
            "SELECT name, priority, IF(interruptable,'Yes','No')FROM Sequences WHERE schedulable = 1 and experimentalLocationId = "\
                + houseLocation + " AND (scenario = 'User Generated' OR scenario = 'Operator' OR scenario = '"\
                + scenario + "') ORDER BY schedulable DESC, priority DESC, RAND()";
  //      qDebug()<<qry;
        if (!query.exec(qry))
        {
            QMessageBox msgBox;
            msgBox.setIcon(QMessageBox::Critical);

            msgBox.setText("DB error - (2) cannot select fom Sequences table!");
            msgBox.exec();
            return false;
        }
    }
    ui->sequenceTableWidget->clear();
    ui->sequenceTableWidget->setColumnCount(4);
    ui->sequenceTableWidget->setRowCount(query.size());
    ui->sequenceTableWidget->verticalHeader()->setVisible(false);

    QStringList labs;
    labs << "Name" << "Priority" << "Int" << "Exec";
    ui->sequenceTableWidget->setHorizontalHeaderLabels(labs);

    int row = 0;

    while (query.next())
    {

        ui->sequenceTableWidget->setItem(row, 0, new QTableWidgetItem(query.value(0).toString()));
        ui->sequenceTableWidget->item(row, 0)->setBackgroundColor(Qt::white);

        QTableWidgetItem *item = new QTableWidgetItem();
        item->setTextAlignment(Qt::AlignCenter);
        item->setText(query.value(1).toString());
        ui->sequenceTableWidget->setItem(row, 1, item);

        ui->sequenceTableWidget->setItem(row, 2, new QTableWidgetItem(query.value(2).toString()));
        ui->sequenceTableWidget->setItem(row, 3, new QTableWidgetItem("No"));
        ui->sequenceTableWidget->item(row, 3)->setBackgroundColor(Qt::red);

        ui->sequenceTableWidget->resizeColumnsToContents();

        row++;

    }

    return true;

}

void MainWindow::on_evaluatePushButton_clicked()
{
    QString sequenceName = ui->sequenceTableWidget->item(ui->sequenceTableWidget->currentRow(), 0)->text();

    bool overallresult = evaluateRules(sequenceName, true);

    QString res;

    if (overallresult)
    {
        res = " -> result is executable.";
        ui->sequenceTableWidget->setItem(ui->sequenceTableWidget->currentRow(), 3, new QTableWidgetItem("Yes"));
        ui->sequenceTableWidget->item(ui->sequenceTableWidget->currentRow(), 3)->setBackgroundColor(Qt::green);
    }
    else
    {
        res = " -> result is NOT executable.";
        ui->sequenceTableWidget->setItem(ui->sequenceTableWidget->currentRow(), 3, new QTableWidgetItem("No"));
        ui->sequenceTableWidget->item(ui->sequenceTableWidget->currentRow(), 3)->setBackgroundColor(Qt::red);
    }

    // logging...

    QString msg = "                 Evaluated sequence: " + sequenceName + res;

    logMessage(msg);

 //   ui->sequenceTableWidget->setCurrentCell(-1, -1);
 //   ui->evaluatePushButton->setEnabled(false);
 //   ui->executePushButton->setEnabled(false);
}



void MainWindow::logMessage(QString msg)
{
    logTableRow++;
    ui->logTableWidget->setRowCount(logTableRow + 1);
    ui->logTableWidget->setItem(logTableRow, 0, new QTableWidgetItem(msg));
    ui->logTableWidget->resizeColumnsToContents();
    ui->logTableWidget->scrollToBottom();
}

void MainWindow::on_sequenceTableWidget_cellDoubleClicked(int row, int column)
{

    on_sequenceTableWidget_cellClicked(row, column);

}

void MainWindow::on_sequenceTableWidget_cellClicked(int row, int column)
{
    column=column;


    QString sequenceName = ui->sequenceTableWidget->item(row, 0)->text();
    QSqlQuery query(db2);
    QString qry =
        "SELECT \
                  CASE ruleType WHEN \"R\" THEN \"Rule\" WHEN \"A\" THEN \"    Action\" END,\
                  ruleOrder, \
                  ruleActionText,\
                  CASE andOrConnector WHEN \"0\" THEN \"*\" WHEN \"2\" THEN \"Or\" WHEN \"1\" THEN \"And\" END,\
                  CASE ruleType WHEN \"R\" THEN \"False\" WHEN \"A\" THEN \" \" END\
                  FROM ActionRules WHERE name = \""
        + sequenceName + "\" AND ExperimentalLocationId = \"" + houseLocation + "\" ORDER BY ruleOrder";

    //  qDebug()<<qry;

    if (!query.exec(qry))
    {

        QMessageBox msgBox;
        msgBox.setIcon(QMessageBox::Critical);

        msgBox.setText("DB error -2- cannot select from ActionRules table!");
        msgBox.exec();

        QApplication::quit();

    }

    ui->SQLtableWidget->clear();
    ui->SQLtableWidget->verticalHeader()->setVisible(false);
    ui->SQLtableWidget->setColumnCount(6);
    ui->SQLtableWidget->setRowCount(query.size());

    QStringList labs;
    labs << "Behaviour" << "Rule/Action" << "Order" << "Description" << "And/Or" << "True/False";
    ui->SQLtableWidget->setHorizontalHeaderLabels(labs);

    int nrow = 0;
    while (query.next())
    {
        ui->SQLtableWidget->setItem(nrow, 0, new QTableWidgetItem(sequenceName));
        ui->SQLtableWidget->setItem(nrow, 1, new QTableWidgetItem(query.value(0).toString()));

        QTableWidgetItem *item = new QTableWidgetItem();
        item->setTextAlignment(Qt::AlignCenter);
        item->setText(query.value(1).toString());
        ui->SQLtableWidget->setItem(nrow, 2, item);
        ui->SQLtableWidget->setItem(nrow, 3, new QTableWidgetItem(query.value(2).toString()));
        ui->SQLtableWidget->setItem(nrow, 4, new QTableWidgetItem(query.value(3).toString()));
//    qDebug()<<query.value(0).toString();

        if (query.value(0).toString() == "Rule")
        {
            ui->SQLtableWidget->setItem(nrow, 4, new QTableWidgetItem(query.value(3).toString()));
            ui->SQLtableWidget->item(nrow, 1)->setBackgroundColor(Qt::gray);
        }
        else
        {
            ui->SQLtableWidget->setItem(nrow, 4, new QTableWidgetItem(""));
            ui->SQLtableWidget->item(nrow, 1)->setBackgroundColor(Qt::lightGray);
        }

        ui->SQLtableWidget->setItem(nrow, 5, new QTableWidgetItem(query.value(4).toString()));
//         qDebug()<<query.value(4).toString();

        if (query.value(4).toString() == "False")
        {
            ui->SQLtableWidget->item(nrow, 5)->setBackgroundColor(Qt::red);
        }

        ui->SQLtableWidget->resizeColumnsToContents();
        nrow++;
    }

    if (ui->startSchedulerPushButton->isEnabled())
    {
        ui->evaluatePushButton->setEnabled(true);
        ui->executePushButton->setEnabled(true);
        on_evaluatePushButton_clicked();
    }
    else
    {
        if (ui->showNonSchedcheckBox->isChecked())
        {
            ui->evaluatePushButton->setEnabled(true);
            ui->executePushButton->setEnabled(true);
            on_evaluatePushButton_clicked();
        }
        else
        {
          ui->evaluatePushButton->setEnabled(false);
          ui->executePushButton->setEnabled(false);
        }
    }

    displayedSequencename = sequenceName;
}

void MainWindow::on_evaluateAllPushButton_clicked()
{
    ui->evaluatePushButton->setEnabled(false);
    ui->executePushButton->setEnabled(false);

    int noRows = ui->sequenceTableWidget->rowCount();

    for (int i = 0; i < noRows; i++)
    {
        QString sequenceName = ui->sequenceTableWidget->item(i, 0)->text();

        QString res;

    //    qDebug()<<"Evaluating " << sequenceName;
    //    qDebug()<<"Selected " << displayedSequencename;

        bool overallresult;

        if (sequenceName == displayedSequencename)
        {
           overallresult = evaluateRules(sequenceName, true);       // this updates the select sequence window as well
        }
        else
        {
            overallresult = evaluateRules(sequenceName, false);
        }

        if (overallresult)
        {
        //    overallresult = overallresult && evaluateResources(sequenceName);
            if (!overallresult)
            {
                ui->sequenceTableWidget->setItem(i, 3, new QTableWidgetItem("No"));
                ui->sequenceTableWidget->item(i, 3)->setBackgroundColor(Qt::blue);
                res = " -> result is NOT executable due to resource lock.";
            }
            else
            {
                ui->sequenceTableWidget->setItem(i, 3, new QTableWidgetItem("Yes"));
                ui->sequenceTableWidget->item(i, 3)->setBackgroundColor(Qt::green);
                res = " -> result is executable.";
            }
        }
        else
        {
            ui->sequenceTableWidget->setItem(i, 3, new QTableWidgetItem("No"));
            ui->sequenceTableWidget->item(i, 3)->setBackgroundColor(Qt::red);
            res = " -> result is NOT executable.";
        }



// logging...

        QString msg = " sequence: " + sequenceName + res;

        logMessage(msg);

        ui->sequenceTableWidget->setCurrentCell(-1, -1);

    }

}

bool MainWindow::evaluateResources(QString sequenceName)
{

//*    if (sequenceName != "") return true; //temporailily skip next section

    QSqlQuery query(db9);

    query.clear();
    qDebug()<< "   Evaluating resources: " << sequenceName;

    query.prepare(
        "SELECT action FROM  ActionRules WHERE ruleType = 'A' AND name = :name AND ExperimentalLocationId = :house");
    query.bindValue(":name", sequenceName);
    query.bindValue(":house", houseLocation);

    db9.database().transaction();
    if (!query.exec())
    {
        qDebug() << "Can't select from actionrules table! (2)" << query.executedQuery();
        qDebug() << "evaluate resource sequence name: " << sequenceName;
        return true;  // means that preconditions do not use resource judgments
    }
    db9.database().commit();

    if (query.size() == 0)   // no actions so always true!
    {
        return true;
    }

    //int recCount = 0;
    // bool overallresult;
    //bool result;

    // overallresult = false;
    //result = false;

    //bool finalResult = true;

    QString resourceName;
    int     robotIdent;
    QString sequenceRecursive;

    while (query.next())
    {
        // get the resource name and the robot number
        QString action;

        QStringList rList;


        rList = query.value(0).toString().split(",");

        resourceName = rList.at(0);
        robotIdent   = rList.at(1).toInt();
        sequenceRecursive = rList.at(2);

//       qDebug()<< "    resource" << resourceName << " robot " << robotIdent << "  Recursive? " << sequenceRecursive;

        if (resourceName == "sequence")
        {
            if (!evaluateResources(sequenceRecursive))
            {
                return false;  // means resource already being used - then this behaviour cannot run!
            }
        }

        if (resourceName == "base" || resourceName == "tray" || resourceName == "torso" || resourceName == "eyes" || resourceName == "head")
        {
            qDebug()<< "   Checking resources: " << sequenceName <<":rName " << resourceName <<" :hId " << houseLocation<<" :rId"<<robotIdent;

            QSqlQuery resourceQuery(db7);

            resourceQuery.clear();

            resourceQuery.prepare(
                "SELECT * FROM Resources WHERE resourceName = :rName AND robotId = :rId AND ExperimentalLocationId = :hId");
            resourceQuery.bindValue(":rName", resourceName);
            resourceQuery.bindValue(":hId", houseLocation);
            resourceQuery.bindValue(":rId", robotIdent);
            db7.database().transaction();
            if (!resourceQuery.exec())
            {
                qDebug() << "Can't select from resources table!" << resourceQuery.executedQuery() << resourceQuery.lastError();
                qDebug() << "-- resource sequence name: " << sequenceName;
                return true;  // means that preconditions do not use resource judgments
            }
            db7.database().commit();

            if (resourceQuery.size() != 0)   // means resource already being used - then this behaviour cannot run!
            {
                return false;
            }

        }




    }

    return true;
}


bool MainWindow::evaluateRules(QString sequenceName, bool display)
{

    QSqlQuery query(db3);
    QSqlQuery queryLogic(db4);


    query.clear();
//   qDebug()<< "evaluate rule sequence name: " << sequenceName;
    query.prepare(
        "SELECT * FROM  ActionRules WHERE ruleType = 'R' AND name = :name AND ExperimentalLocationId = :house ORDER BY ruleOrder");
    query.bindValue(":name", sequenceName);
    query.bindValue(":house", houseLocation);

    db3.database().transaction();
    if (!query.exec())
    {
        qDebug() << "Can't select from actionrules table! (1)" << query.executedQuery();
        qDebug() << query.lastError();
        qDebug() << "evaluate rule sequence name: " << sequenceName;
        return false;
    }
    db3.database().commit();


    if (query.size() == 0)   // no rules so always true!
    {
        return true;
    }

    QString prevANDORSwitch = "undefined";
    QString ANDORSwitch;

    int recCount = 0;
    bool overallresult, result;

    overallresult = result = false;

    bool finalResult = true;

    while (query.next())
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

            db4.database().transaction();
            if (!queryLogic.exec(query.value(6).toString()))
            {
                qDebug() << "Can't execute rule from actionrules table!" << queryLogic.executedQuery();
                qDebug() << "Rule is: " << query.value(6).toString();
                return false;
            }
            db3.database().commit();

            int row = recCount - 1;

            if (queryLogic.size() > 0)
            {
                if (display)
                {
                    ui->SQLtableWidget->setItem(row, 5, new QTableWidgetItem("True"));
                    ui->SQLtableWidget->item(row, 5)->setBackgroundColor(Qt::green);
                }
                result = true;
                //         qDebug()<<" <-- true";
            }
            else
            {
                if (display)
                {
                    ui->SQLtableWidget->setItem(row, 5, new QTableWidgetItem("False"));
                    ui->SQLtableWidget->item(row, 5)->setBackgroundColor(Qt::red);
                }
                result = false;
                //       qDebug()<<" <-- false";
            }

            // first record
            if (recCount == 1)
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
//   if (runWithROS)
//   {
//      robot->stop();
//   }



    QString sequenceName = ui->sequenceTableWidget->item(ui->sequenceTableWidget->currentRow(), 0)->text();

    executionResult = 0;

    if (evaluateRules(sequenceName, false))
    {
         runSequence(sequenceName, 0, "No", ui->sequenceTableWidget->currentRow());
    }
    else
    {
       executionResult = BEHAVIOUR_NOT_EXECUTABLE;
    }
    checkExecutionResult();
}

void MainWindow::checkStopExecution()
{
    if (stopExecution)
    {
        throw threadInterruptedException();
    }
}

int MainWindow::retryMessage(QString msg )
{
    QByteArray byteArray = msg.toUtf8();
    const char* cString = byteArray.constData();

    int ret = QMessageBox::warning(ui->centralWidget, tr("Care-O-Bot Scheduler"),
                                   tr(cString),
                                   QMessageBox::No | QMessageBox::Yes);

    if (ret == QMessageBox::Yes)
    {
        return 1;
    }
    else
    {
        return 0;
    }

}


int MainWindow::executeSequence(QString sequenceName, bool display)
{

    display=display;

    qDebug() << "=========================================================== " << rc++;;
    if (runWithROS)
    {
        checkStopExecution();
    }
    QString indentSpaces = "";

    for (int i = 0; i < indent; i++)
    {
        indentSpaces += " ";
    }

    int returnResult = NO_PROBLEMS;

    bool overallresult = evaluateRules(sequenceName, display);
    qDebug() << indentSpaces + "               Evaluated " << sequenceName << " result " << overallresult;


    // no longer executable!
    if (!overallresult)
    {
      qDebug()<<indentSpaces + "               " + sequenceName << " not executable anymore!";
   //   returnResult = RULES_INVALID;
        return NO_PROBLEMS;

    }

    // get the set of actions for this sequence
    QSqlQuery query(db1);
    query.clear();
    query.prepare(
        "SELECT action FROM  ActionRules WHERE name = :name AND ruleType = \"A\" and ExperimentalLocationId = :house ORDER BY ruleOrder");
    query.bindValue(":name", sequenceName);
    query.bindValue(":house", houseLocation);

    if (!query.exec())
    {
        qDebug()<< query.lastError();
        qDebug()<< query.lastQuery();
        qDebug()<< query.executedQuery();

        returnResult = ACTIONRULES_DB_ERROR_SELECT;
        qDebug() << "Execute Sequence Name: " << sequenceName;
        qDebug() << query.lastError();
        return returnResult;
    }

    // for each action - execute it via the script server
    while (query.next())
    {
        // unpack query
        QString str = query.value(0).toString();
        qDebug() << indentSpaces + "               " + str;

        QString outcome = "";

        cname = str.section(',', 0, 0);
        rname = str.section(',', 1, 1);
        pname = str.section(',', 2, 2);
        pname1 = str.section(',', 3, 3);
        wait = str.section(',', 4, 4);

        // lock the Sienna GUI if doing anything but sleeping
        QSqlQuery Lockquery(db7);
        Lockquery.clear();

        if (cname == "tray" || cname == "base" || cname == "torso" || cname == "eyes" || cname == "head"
                || cname == "arm")
        {
            Lockquery.prepare("UPDATE Sensors SET value = 1 WHERE sensorId = 1001");
            //sleep(5);
            //qDebug()<<"Set progress to 1";
        }
        else
        {
            Lockquery.prepare("UPDATE Sensors SET value = 0 WHERE sensorId = 1001");
            //qDebug()<<"Set progress to 0";
        }

        if (!Lockquery.exec())
        {
            returnResult = SENSORS_DB_ERROR_UPDATE;
            qDebug()<<Lockquery.lastError();
            qDebug()<<"When Updating Sensors";

            return returnResult;
        }

        // set blocking to always true regardless of wait until Nathan fixed script server issue

        bool blocking = true;

        // if (wait == "wait")
        // {
        //      blocking = true;
        // }

        string returnRes = "SUCCEEDED";

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
            if (runWithROS)
            {
                checkStopExecution();
                robot->say(pname.toStdString(), "", blocking);
            }

            qDebug() << indentSpaces + "               Say " << pname;
            returnResult = NO_PROBLEMS;

        }

        // -----------
        // make the robot play something
        // -----------
        if (cname == "play")
        {
            if (runWithROS)
            {
                checkStopExecution();
                robot->play(pname.toStdString(), blocking);
            }

            qDebug() << indentSpaces + "               Play " << pname;
            returnResult = NO_PROBLEMS;

        }

        // -----------
        // set the light colour - format:   "" set_light red ""
        // -----------
        if (cname == "light")
        {
            if (runWithROS)
            {
                checkStopExecution();
                robot->setLight(pname.toStdString());
            }

            qDebug() << indentSpaces + "               Set light to " << pname;
            returnResult = NO_PROBLEMS;
        }

        // -----------
        // add action history async
        // -----------
        if (cname=="save")
        {
            if (runWithROS)
            {
                checkStopExecution();
    //            actionHistory->addHistoryCompleteAsync(sequenceName.toStdString());
                qDebug() << "action history call ** get latest version from Nathan";
            }
            qDebug() << indentSpaces + "               Save History Complete Async ";
            returnResult = NO_PROBLEMS;
        }

        // -----------
        // make the robot go to sleep - format: sleep "" "" 10
        // -----------
        if (cname == "sleep")
        {
            double p = pname.toDouble();
            p = p * 1000;
            int t = p;
            //   qDebug()<<t;
            if (runWithROS)
            {
                checkStopExecution();
                robot->sleep(t);
            }
            else
            {
                int n = pname.toInt();

                n*= 1000000;
         //       qDebug()<<n<<" "<<pname;
                usleep(n);
            }

            qDebug() << indentSpaces + "               Sleep for " << pname << " seconds";
            returnResult = NO_PROBLEMS;
        }

        // -----------
        // actuation - format: move base [x,y,theta] ""
        //                     move tray up ""
        //                     move torso nod ""
        //                     move eyes forward ""
        //                     move head left ""
        // -----------
        if (cname == "tray" || cname == "base" || cname == "torso" || cname == "eyes" || cname == "head"
                || cname == "arm")
        {
            // resource lock
            /*
            qDebug()<<"    Setting resource lock for:" << cname << " " << sequenceName << " " << rname << " " << houseLocation;

            QSqlQuery resourceQuery(db7);


            resourceQuery.clear();

            resourceQuery.prepare("INSERT INTO Resources VALUES (:resourceName, :behaviourName, :robotId, :experimentalLocationId)");

            resourceQuery.bindValue(":resourceName", cname);
            resourceQuery.bindValue(":behaviourName", sequenceName);
            resourceQuery.bindValue(":robotId", rname);
            resourceQuery.bindValue(":experimentalLocationId", houseLocation);

            db7.database().transaction();
            qDebug() << "About to exec";
            if (!resourceQuery.exec())
            {
                qDebug()<<"Insert failed " << resourceQuery.lastQuery();
            }

            db7.database().commit();

            */

            // logic for base is: send x,y,theta from db location, if theta spec'ed by user use that
            // if go to user (999) send string "userLocation" and let the context system sort it out.
            if (cname == "base")
            {
                pname.remove('[');
                pname.remove(']');

                double degrees = pname.section(":", 2, 2).toDouble();  // we hold on the database in degrees
                double radians = degrees / 180.0 * 3.142;              // the command must be in radians
                QString rd;
                rd.setNum(radians);

                std::vector<double> pos(3);

                pos.at(0) = pname.section(":", 0, 0).toDouble();
                pos.at(1) = pname.section(":", 1, 1).toDouble();
                pos.at(2) = radians;

                // but use the database location if available
                if (pname1 != "")
                {
                    if (pname1 == "999")
                    {
                        if (runWithROS)
                        {
                            checkStopExecution();
                            returnRes = robot->setComponentState(cname.toStdString(), "userLocation", blocking);
                            outcome = QString::fromStdString(returnRes);
                        }

                        qDebug() << indentSpaces + "               Set " << cname << " to " << pname << " " << wait;

                        if (outcome != "SUCCEEDED" && runWithROS)
                        {
                            qDebug() << indentSpaces + "               " << outcome;

                             QString msg = "Failure in userlocation navigation ";
                             msg = msg + cname + " " + outcome + " " + " - retrying...";

                             qDebug()<<msg;

                             returnRes = robot->setComponentState(cname.toStdString(), pname.toStdString(), blocking);
                             outcome = QString::fromStdString(returnRes);

                             if (outcome != "SUCCEEDED")
                             {
                                qDebug()<<"Problem with base to user location - giving up!";
                                returnResult = SCRIPTSERVER_EXECUTION_FAILURE;
                             }
                             else
                             {
                                returnResult = NO_PROBLEMS;
                             }
                        }
                        else
                        {
                           returnResult = NO_PROBLEMS;
                        }
                    }
                    else
                    {
                        QSqlQuery locationQuery(db7);
                        locationQuery.clear();
                        locationQuery.prepare(
                            "SELECT xCoord,yCoord,orientation from Locations WHERE locationId = :locn LIMIT 1");
                        locationQuery.bindValue(":locn", pname1);

                        if (locationQuery.exec())
                        {
                            while (locationQuery.next())
                            {
                                pos.at(0) = locationQuery.value(0).toDouble();
                                pos.at(1) = locationQuery.value(1).toDouble();
                                pos.at(2) = radians;
                            }
                        }
                        else
                        {
                            qDebug() << "Location query failed - using user given location vector";
                        }

                        qDebug() << indentSpaces + "               Set " << cname << " to " << pname1 << " " << wait;

                        if (runWithROS)
                        {
                            checkStopExecution();
                            qDebug() << pos[0];
                            qDebug() << pos[1];
                            qDebug() << pos[2];
                            returnRes = robot->setComponentState(cname.toStdString(), pos, blocking);
                            outcome = QString::fromStdString(returnRes);
                        }

                        if (outcome != "SUCCEEDED" && runWithROS)
                        {
                            qDebug() << indentSpaces + "               " << outcome;

                            QString msg;
                            msg = "Failure in navigation to location ";
                            msg += " ";
                            msg += outcome;
                            msg += " ";
                            msg += " - retrying...";

                            qDebug()<<msg;

                            returnRes = robot->setComponentState(cname.toStdString(), pos, blocking);
                            outcome = QString::fromStdString(returnRes);

                            if (outcome != "SUCCEEDED")
                            {
                                    qDebug()<<"Problem with navigation to location - giving up!";
                                    returnResult = SCRIPTSERVER_EXECUTION_FAILURE;
                            }
                            else
                            {
                               returnResult = NO_PROBLEMS;
                            }
                         }
                         else
                         {
                            returnResult = NO_PROBLEMS;
                         }
                    }
                }
            }

            // differences between cob 3,2 and cob 3.5/6
            if (cname == "tray" || cname == "torso" || cname == "eyes" || cname == "head" || cname == "arm")
            {
                if (pname == "store")
                {
                    pname = "lowered";
                }
                if (pname == "deliverup")
                {
                    pname = "raised";
                }

                if (cname == "arm")
                {
                    pname = pname + ":" + pname1;
                }

                if (runWithROS)
                {
                    checkStopExecution();
                    returnRes = robot->setComponentState(cname.toStdString(), pname.toStdString(), blocking);
                    outcome = QString::fromStdString(returnRes);
                }

                qDebug() << indentSpaces + "               Set " << cname << " to " << pname << " " << wait;

                if (outcome != "SUCCEEDED" && runWithROS)
                {

                    qDebug() << indentSpaces + "               " << outcome;

                    QString msg = "Failure in ";
                    msg = msg + cname + " " + outcome + " Retrying";

                    qDebug()<<msg;

                    returnRes = robot->setComponentState(cname.toStdString(), pname.toStdString(), blocking);
                    outcome = QString::fromStdString(returnRes);

                    if (outcome != "SUCCEEDED")
                    {
                       qDebug()<<"Problem with component - giving up!";
                       returnResult = SCRIPTSERVER_EXECUTION_FAILURE;
                    }
                    else
                    {
                        returnResult = NO_PROBLEMS;
                    }
                }
                else
                {
                    returnResult = NO_PROBLEMS;
                }
            }

            // clear resource lock
            /*
            qDebug()<<"    Clearing resource lock for:" << cname << " " << sequenceName << " " << rname << " " << houseLocation;

            resourceQuery.clear();

            resourceQuery.prepare("DELETE FROM Resources Where resourceName = :resourceName AND behaviourName = :behaviourName AND robotId = :robotId and experimentalLocationId = :experimentalLocationId");

            resourceQuery.bindValue(":resourceName", cname);
            resourceQuery.bindValue(":behaviourName", sequenceName);
            resourceQuery.bindValue(":robotId", rname);
            resourceQuery.bindValue(":experimentalLocationId", houseLocation);

            db7.database().transaction();
            if (!resourceQuery.exec())
            {
                qDebug()<<"Delete failed " << resourceQuery.lastQuery() << " " << resourceQuery.lastError();
            }
            db7.database().commit();
            */
        }

        // -----------
        // The user GUI - doesn't use script server
        // -----------
        // set the wait flag in db and await response
        if (cname == "GUI")
        {

            QSqlQuery GUIquery(db5);
            GUIquery.clear();
            GUIquery.prepare("UPDATE userInterfaceGUI SET guiMsgResult = NULL WHERE name = :name ");
            GUIquery.bindValue(":name", sequenceName);

            if (!GUIquery.exec())
            {
                returnResult = USERINTGUI_DB_ERROR_SELECT;
                qDebug()<<"userInterfaceGUI table update";
                qDebug() << GUIquery.lastError();
                return returnResult;
            }

            bool awaitingUserResponse = true;
            int numSeconds = 0;

            while (awaitingUserResponse)
            {
                if (runWithROS)
                {
                    checkStopExecution();
                }
                numSeconds++;

                // what should we do if the user doesn't respond?
                if (numSeconds > 60)
                {
                    //returnResult = USER_TIMEOUT;
                    //return returnResult;
                    qDebug() << "User hasn't responded to GUI...";
                    numSeconds = 50;   // now inform every 10 seconds
                }

                sleep(1);
                //qDebug() << numSeconds;
                GUIquery.clear();
                GUIquery.prepare("SELECT guiMsgResult FROM userInterfaceGUI WHERE name = :name ");
                GUIquery.bindValue(":name", sequenceName);

                if (!GUIquery.exec())
                {
                    returnResult = USERINTGUI_DB_ERROR_SELECT;
                    qDebug()<<"guiMsgResult table select";
                    qDebug() << GUIquery.lastError();
                    return returnResult;
                }

                GUIquery.next();
                QString resultFromUser = GUIquery.value(0).toString();

                //     qDebug()<<resultFromUser;
                if (resultFromUser == "0" || resultFromUser == "") // no response
                    continue;

                awaitingUserResponse = false;       // got a response

                //    qDebug()<<pname;
                //    qDebug()<<pname.section("@",resultFromUser.toInt()-1,resultFromUser.toInt()-1);

                indent += 3;
                returnResult = mainW->executeSequence(
                                   pname.section("@", resultFromUser.toInt() - 1, resultFromUser.toInt() - 1), false);
                indent -= 3;
            }
        }

        // -----------
        // set logical goals and conditions - doesn't use script server
        // -----------
        if (cname == "cond")
        {
            QSqlQuery Goalquery(db6);
            Goalquery.clear();
            Goalquery.prepare(
                "INSERT INTO SensorLog (timestamp,sensorId,room,channel,value,status) VALUES (NOW(),:goalId,'','',:value,:truefalse)");
            Goalquery.bindValue(":value", pname1);
            Goalquery.bindValue(":goalId", pname);

            QString TF = "true";

            if (pname1 == "0")
            {
                TF = "false";
            }

            Goalquery.bindValue(":truefalse", TF);


            if (!Goalquery.exec())
            {
                returnResult = ACTIONGOALS_DB_ERROR_UPDATE;
                qDebug() << Goalquery.lastError();
                qDebug()<<"Inserting to sensor log";
            }
        }

        // expressions on the Siena GUI - doesn't use script server
        if (cname == "expression")
        {
            QSqlQuery Goalquery(db10);
            Goalquery.clear();
            db10.database().transaction();
            Goalquery.prepare("UPDATE GUIexpression SET ison = 0 WHERE ison = 1");
            if (!Goalquery.exec())
            {
                returnResult = ACTIONGOALS_DB_ERROR_UPDATE;
                qDebug() << Goalquery.lastError();
                qDebug()<<"updating GUIexpression ison 0";
            }

            qDebug() << indentSpaces + "               Updating expression to " << pname1;
            Goalquery.prepare("UPDATE GUIexpression SET ison = 1 WHERE id = :id");
            Goalquery.bindValue(":id", pname);
            if (!Goalquery.exec())
            {
                returnResult = ACTIONGOALS_DB_ERROR_UPDATE;
                qDebug() << Goalquery.lastError();
                qDebug()<<"updating GUIexpression ison 1";
            }

            db10.database().commit();
        }

        // action possibilities on the Siena GUI - doesn't use script server
        if (cname == "APoss")
        {
            double currentLikelyhood;

            QSqlQuery Goalquery(db6);
            Goalquery.clear();
            db6.database().transaction();
            Goalquery.prepare("SELECT likelihood from ActionPossibilities where apid = :apid LIMIT 1");
            Goalquery.bindValue(":apid", pname);

            if (!Goalquery.exec())
            {
                returnResult = ACTIONGOALS_DB_ERROR_UPDATE;
                qDebug() << Goalquery.lastError();
                qDebug()<<"Selecting from ActionPossibilities";
            }

            while (Goalquery.next())
            {
                currentLikelyhood = Goalquery.value(0).toDouble();
            }

            //qDebug()<<pname<<" " <<pname1;
            //qDebug()<<currentLikelyhood;
            currentLikelyhood = pname1.toDouble();
            //qDebug()<<currentLikelyhood;

            Goalquery.prepare("UPDATE ActionPossibilities SET likelihood = :like where apid = :apid");
            Goalquery.bindValue(":apid", pname);
            Goalquery.bindValue(":like", currentLikelyhood);

            if (!Goalquery.exec())
            {
                returnResult = ACTIONGOALS_DB_ERROR_UPDATE;
                qDebug() << Goalquery.lastError();
                qDebug()<<"Updating ActionPossibilities";
            }

            db6.database().commit();
        }

        // -----------
        // recursive calls to this routine
        //-----------
        if (cname == "sequence")
        {
            //       qDebug()<<indentSpaces + "               Call made to " << pname;
            indent += 3;
            if (runWithROS)
            {
                checkStopExecution();
            }
            returnResult = mainW->executeSequence(pname, false);
            indent -= 3;
        }
    }  // end of while

    //qDebug()<<"               Sequence: " + sequenceName + " finished";
    return returnResult;
}

void MainWindow::on_COBTestPushButton_clicked()
{
    if (!fillSequenceTable(scenario))
    {
        QApplication::quit();
    }
}

void MainWindow::on_startSchedulerPushButton_clicked()
{
    logMessage("Starting scheduler...");

    ui->COBTestPushButton->setEnabled(false);
    ui->showNonSchedcheckBox->setEnabled(false);
    ui->showNonSchedcheckBox->setChecked(false);

    fillSequenceTable(scenario);

    currentlyExecutingSequence = "";
    currentlyExecutingPriority = -1;
    currentlyExecutingCanInterrupt = "Yes";
    currentlyExecutingRow = 0;

    timer.start(loopSpeed);  // this controls how fast the behaviour list is checked

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

    int rows = ui->sequenceTableWidget->rowCount();

    for (int i = 0; i < rows; i++)
    {
        ui->sequenceTableWidget->item(i, 0)->setBackgroundColor(Qt::white);
    }

}

void MainWindow::doSchedulerWork()
{
    if (sched->isRunning())
        qDebug() << "           Thread " << currentlyExecutingSequence << " is ----- Running -----";
    if (sched->isFinished())
        qDebug() << "           *** Thread terminated ***";

    if (executionResult != 0)
    {
        stopSequence(currentlyExecutingSequence);
        checkExecutionResult();
 //       QApplication::quit();    joe
    }

//    logMessage("Evaluating sequence list...");

    QString readySequence = "";
    int readyPriority = -1;
    QString readyCanInterrupt = "No";
    int readyRow = 0;

//    fillSequenceTable(scenario);

    int rows = ui->sequenceTableWidget->rowCount();

    bool overallresult = false;
    bool result = false;

    for (int i = 0; i < rows; i++)
    {
        QString sequenceName = ui->sequenceTableWidget->item(i, 0)->text();
        int priority = ui->sequenceTableWidget->item(i, 1)->text().toInt();
        QString CanInterrupt = ui->sequenceTableWidget->item(i, 2)->text();

        ui->sequenceTableWidget->item(i, 0)->setBackgroundColor(Qt::yellow);

   //     qDebug() << "Evaluating: " << sequenceName;

        if (sequenceName == displayedSequencename)
        {
           result = evaluateRules(sequenceName, true);       // this updates the select sequence window as well
        }
        else
        {
            result =evaluateRules(sequenceName, false);
        }


        if (result)
        {
    //        result = result && evaluateResources(sequenceName);
            if (!result)
            {
                ui->sequenceTableWidget->setItem(i, 3, new QTableWidgetItem("No"));
                ui->sequenceTableWidget->item(i, 3)->setBackgroundColor(Qt::blue);
            }
            else
            {
                ui->sequenceTableWidget->setItem(i, 3, new QTableWidgetItem("Yes"));
                ui->sequenceTableWidget->item(i, 3)->setBackgroundColor(Qt::green);
            }
        }
        else
        {
            ui->sequenceTableWidget->setItem(i, 3, new QTableWidgetItem("No"));
            ui->sequenceTableWidget->item(i, 3)->setBackgroundColor(Qt::red);
        }



//*       ui->sequenceTableWidget->repaint();

        if (result)
        {
            qDebug() << "    is runnable ***";
            // because the list is in priority order the first valid rule is chosen
            if (!overallresult)
            {
                qDebug() << "           is highest priority ***";
                overallresult = true;
                readySequence = sequenceName;
                readyPriority = priority;
                readyCanInterrupt = CanInterrupt;
                readyRow = i;
            }

        }
    }

    if (!overallresult)
        return;

    logMessage("Ready to execute: " + readySequence);
    qDebug() << "           *** Ready to run: " << readySequence;

// nothing currently executing
    if (currentlyExecutingSequence == "")
    {
        qDebug() << "           *** starting to run: " << readySequence;
        runSequence(readySequence, readyPriority, readyCanInterrupt, readyRow);         // run the ready sequence
    }
    else
    {
// ready and higher priority
        if (readyPriority > currentlyExecutingPriority && currentlyExecutingCanInterrupt == "Yes")
        {
            qDebug() << "           ***!!! " << readySequence << " usurped " << currentlyExecutingSequence
                     << " !!!****";
            logMessage(readySequence + " usurped " + currentlyExecutingSequence);
            stopSequence(currentlyExecutingSequence);
            runSequence(readySequence, readyPriority, readyCanInterrupt, readyRow);
        }
        else
        {
            if (readySequence == currentlyExecutingSequence)
            {
                qDebug() << "           *** " << readySequence << " in progress << ****";
                logMessage(readySequence + " in progress...");
            }
        }
    }
    qDebug()<<"end doSchedulerWork";
}
void MainWindow::stopSequence(QString sequenceName)
{
    logMessage("Stopping: " + currentlyExecutingSequence);
    ui->sequenceTableWidget->item(currentlyExecutingRow, 1)->setBackgroundColor(Qt::white);

    QSqlQuery GUIquery(db7);  // kill the gui if necessary
    GUIquery.clear();
    GUIquery.prepare("UPDATE userInterfaceGUI SET guiMsgResult = 0 WHERE name = :name ");
    GUIquery.bindValue(":name", sequenceName);
    GUIquery.exec();

// First try to cleanly exit the thread
    stopExecution = true;
    if (runWithROS)
    {
        robot->stop();
    }
    qDebug() << "Stopping sequence thread";
    if (!sched->wait(5000))
    {
// Fallback to terminating the thread, but we're going to have to rebuild the robot class too,
// since it's been left in an indeterminite state
        qDebug() << " !!!*** Thread did not close in time, terminating, scheduler may be in an unstable state ***!!!";
        sched->terminate();
    }
    else
    {
        qDebug() << "Thread cleanly stopped.";
    }

    currentlyExecutingSequence = "";

    qDebug()<<"end doSchedulerWork";
}

void MainWindow::runSequence(QString sequenceName, int priority, QString CanInterrupt, int row)
{
    logMessage("Executing: " + sequenceName);
//*   ui->sequenceTableWidget->item(row, 1)->setBackgroundColor(Qt::cyan);

    currentlyExecutingSequence = sequenceName;
    currentlyExecutingPriority = priority;
    currentlyExecutingCanInterrupt = CanInterrupt;
    currentlyExecutingRow = row;

    stopExecution = false;
    sched->start();
    //checkExecutionResult();
    qDebug()<<"end runSequence";
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
        msgBox.setText("DB error -1- cannot select from ActionRules table!");
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
        msgBox.setText("DB error - cannot select/update from UserInterfaceGUI table!");
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

    case SENSORS_DB_ERROR_UPDATE:
        msgBox.setIcon(QMessageBox::Critical);
        msgBox.setText("DB error - cannot update Sensors table!");
        msgBox.exec();
        break;


    case BEHAVIOUR_NOT_EXECUTABLE:
        msgBox.setIcon(QMessageBox::Warning);
        msgBox.setText("That behaviour is not currently executable!");
        msgBox.exec();
        break;


    }
}
void MainWindow::on_showNonSchedcheckBox_toggled(bool checked)
{
    ui->startSchedulerPushButton->setEnabled(!checked);
    fillSequenceTable(scenario);
}

bool MainWindow::openDatabase(QString dbName, QString host, QString user, QString pw, QString dbase, QSqlDatabase& db)
{
    db = QSqlDatabase::addDatabase("QMYSQL", dbName);

    db.setHostName(host);
    db.setDatabaseName(dbase);
    db.setUserName(user);
    db.setPassword(pw);

    dbOpen = db.open();

    if (!dbOpen)
    {
        QMessageBox msgBox;
        msgBox.setIcon(QMessageBox::Critical);
        msgBox.setText("Database error - login problem - see console log! Connection: " + dbName);
        msgBox.exec();

        qCritical("Cannot open database: %s (%s)", db0.lastError().text().toLatin1().data(),
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

bool MainWindow::openAllDatabaseConnections(QString host, QString user, QString pw, QString dbase)
{

    if (!openDatabase("db0", host, user, pw, dbase, db0))
    {
        return false;
    }

    if (!openDatabase("db1", host, user, pw, dbase, db1))
    {
        return false;
    }

    if (!openDatabase("db2", host, user, pw, dbase, db2))
    {
        return false;
    }

    if (!openDatabase("db3", host, user, pw, dbase, db3))
    {
        return false;
    }

    if (!openDatabase("db4", host, user, pw, dbase, db4))
    {
        return false;
    }

    if (!openDatabase("db5", host, user, pw, dbase, db5))
    {
        return false;
    }

    if (!openDatabase("db6", host, user, pw, dbase, db6))
    {
        return false;
    }

    if (!openDatabase("db7", host, user, pw, dbase, db7))
    {
        return false;
    }

    if (!openDatabase("db8", host, user, pw, dbase, db8))
    {
        return false;
    }

    if (!openDatabase("db9", host, user, pw, dbase, db9))
    {
        return false;
    }

    if (!openDatabase("db10", host, user, pw, dbase, db10))
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
    db8.close();
    db9.close();
    db10.close();

}

void MainWindow::on_enableDebugPushButton_clicked()
{

    debugEnabled = !debugEnabled;

    if (debugEnabled)
    {
        ui->enableDebugPushButton->setStyleSheet("QPushButton { background-color : green; color : white; }");
        ui->GUIgroupBox->setEnabled(true);
    }
    else
    {
        ui->enableDebugPushButton->setStyleSheet("QPushButton { background-color : transparent; color : black; }");
        ui->GUIgroupBox->setEnabled(false);
    }
}

void MainWindow::on_GUIradioButton1_clicked()
{
    updateGUI(1);

}
void MainWindow::on_GUIradioButton2_clicked()
{
    updateGUI(2);
}
void MainWindow::on_GUIradioButton3_clicked()
{
    updateGUI(3);
}
void MainWindow::on_GUIradioButton4_clicked()
{
    updateGUI(4);
}

void MainWindow::updateGUI(int option)
{
    QSqlQuery GUIquery(db7);
    GUIquery.clear();
    GUIquery.prepare("UPDATE userInterfaceGUI SET guiMsgResult = :opt WHERE name = :name");
    GUIquery.bindValue(":opt", option);
    GUIquery.bindValue(":name", currentlyExecutingSequence);
    GUIquery.exec();
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

    qDebug() << ">>>>>>>>> Testing planner....";

    QString fileName = "/home/joe/JSHOP2/CareOBot/problem";

    QFile file(fileName);

    if (!file.open(QIODevice::WriteOnly))
    {
        qDebug() << "Cannot open file for writing: " << qPrintable(file.errorString());
        return;
    }

    QTextStream out(&file);

    out << "; this is an auto-generated file defining a navigation problem for the Care-O-Bot" << endl;
    out << ";" << endl;
    out
            << "; - The current state of robot and its location adjacency conditions are extracted from the Robot House database"
            << endl;
    out << ";" << endl;
    out << "" << endl;
    out << "(defproblem problem navigation" << endl;
    out << "" << endl;
    out << "; we define the state of the house and the robot" << endl;
    out << "" << endl;
    out << "  (" << endl;

    query.clear();
    query.prepare(
        "SELECT L.name, R.locationId from Robot R, Locations L\
                                    WHERE R.locationId = L.locationId AND robotId = 0");

    if (!query.exec())
    {
        qDebug() << "Can't select from robot table!" << query.executedQuery();
        return;
    }

    while (query.next())
    {
        out << "     (at care-o-bot " << query.value(0).toString().simplified().replace(" ", "")
            << query.value(1).toString() << "); define current location of robot" << endl;
    }

    out << "" << endl;
    out << "; define all the locations" << endl;
    out << "" << endl;

    query.clear();
    query.prepare("SELECT name, locationId FROM  Locations where ValidRobotLocation = 1");

    if (!query.exec())
    {
        qDebug() << "Can't select from Locations table!" << query.executedQuery();
        return;
    }

    while (query.next())
    {
        out << "     (room " << query.value(0).toString().simplified().replace(" ", "") << query.value(1).toString()
            << ")" << endl;
    }

    out << "" << endl;

    query.clear();

    query.prepare(
        "SELECT  L.name,  A.locationFrom,\
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
    out << "" << endl;

    while (query.next())
    {

        out << "     (adjacentTo " << query.value(0).toString().simplified().replace(" ", "")
            << query.value(1).toString() << " "\
            << query.value(2).toString().simplified().replace(" ", "")
            << query.value(3).toString() << " "\
            << query.value(4).toString() << ")" << endl;
        out << "     (adjacentTo " << query.value(2).toString().simplified().replace(" ", "")
            << query.value(3).toString() << " "\
            << query.value(0).toString().simplified().replace(" ", "")
            << query.value(1).toString() << " "\
            << query.value(4).toString() << ")" << endl;
    }

    out << "" << endl;
    out << "  )" << endl;
    out << "" << endl;
    out << "; now define the goal" << endl;
    out << "" << endl;
    out << "  (" << endl;
    out << "" << endl;

// create the goal

    query.clear();
    query.prepare("SELECT L.name, L.locationId from Locations L WHERE L.locationId = " + destination);

    if (!query.exec())
    {
        qDebug() << "Can't select from locations table!" << query.executedQuery();
        return;
    }

    while (query.next())
    {
        out << "     (navigate care-o-bot " << query.value(0).toString().simplified().replace(" ", "")
            << query.value(1).toString() << "); define current location of robot" << endl;
    }

    out << "" << endl;

    out << "  )" << endl;
    out << ")" << endl;

    file.close();

    FILE *in;

    char buff[512];

    if (!(in = popen("/home/joe/JSHOP2/CareOBot/doPlanning.sh", "r")))
    {
        qDebug() << "Can't open output for popen call!";
        return;
    }

    while (fgets(buff, sizeof(buff), in) != NULL)
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

void MainWindow::on_speedSpinBox_valueChanged(int arg1)
{
    loopSpeed = arg1;
}
