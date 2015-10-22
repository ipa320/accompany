#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <QInputDialog>
#include <QDebug>
#include <QMessageBox>

QSqlDatabase db;
bool dbOpen;
int experimentLocation;
QString experimentLocationString;
QString sessionUser;
QString languageId;
int deleteCandidate;

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

    ui->userLabel->setText(user + ":" + host);


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


    QSqlQuery query("SELECT ExperimentalLocationId,Sessionuser  FROM SessionControl WHERE SessionId = 1 LIMIT 1");

    if (query.next())
    {
       experimentLocation = query.value(0).toInt();
       experimentLocationString = query.value(0).toString();
       sessionUser = query.value(1).toString();
    }
    else
    {
        QMessageBox msgBox;
        msgBox.setIcon(QMessageBox::Critical);

        msgBox.setText("Cant find session control table!");
        msgBox.exec();
        closeDownRequest = true;
        return;
    }

    // get language

    query = "SELECT languageId FROM Users where userId = " + sessionUser + " LIMIT 1";

    if (query.next())
    {
       languageId = query.value(0).toString();
    }
    else
    {
        QMessageBox msgBox;
        msgBox.setIcon(QMessageBox::Critical);

        msgBox.setText("Cant find users table!");
        msgBox.exec();
        closeDownRequest = true;
        return;
    }

    fillAPTextCombo();
    fillAPPredicates();
    fillDisplayArea();

 }

void MainWindow::fillAPTextCombo()
{
    QString seqQuery;
    QSqlQuery query;

    seqQuery = "SELECT messageId,message FROM Messages where languageId = " + languageId;

    query = seqQuery;

    ui->APTextComboBox->clear();
    ui->APPhraseComboBox->clear();

    while(query.next())
    {
        ui->APTextComboBox->addItem("::" + query.value(0).toString() + ":: " + query.value(1).toString());
        ui->APPhraseComboBox->addItem("::" + query.value(0).toString() + ":: " + query.value(1).toString());
    }


}

void MainWindow::fillAPPredicates()
{
    QString seqQuery;
    QSqlQuery query;

    seqQuery = "SELECT sensorId,name FROM Sensors where sensorId > 499  and sensorId < 600";

    query = seqQuery;

    ui->APPredComboBox->clear();


    while(query.next())
    {
        ui->APPredComboBox->addItem("::" + query.value(0).toString() + ":: " + query.value(1).toString());
    }


}



void MainWindow::on_clearAPPushButton_clicked()
{
    if (QMessageBox::No == QMessageBox::question(this, tr("Action Possibility Creator"),
                                    tr("Do you really want to clear Action Possibility Thresholds?"),
                                    QMessageBox::Yes | QMessageBox::No, QMessageBox::No))
            return;

    QSqlQuery Goalquery(db);

    Goalquery.clear();

    db.database().transaction();

      Goalquery.prepare("UPDATE ActionPossibilities SET likelihood = 0");
      if (!Goalquery.exec())
      {
          QMessageBox msgBox;
          msgBox.setIcon(QMessageBox::Critical);

          msgBox.setText("Cant update action possibilities table!");
          msgBox.exec();
          closeDownRequest = true;
          return;
      }

    db.database().commit();

    fillDisplayArea();
}

void MainWindow::on_APCreatePushButton_clicked()
{
    QSqlQuery query;

    query.prepare("SELECT MAX(apId) FROM ActionPossibilities");

    if (!query.exec())
    {
        qDebug() << query.lastQuery();

        QMessageBox msgBox;
        msgBox.setIcon(QMessageBox::Critical);

        msgBox.setText("Can't select from ActionPossibilities table?");
        msgBox.exec();

        qCritical("Cannot add/update: %s (%s)",
                  db.lastError().text().toLatin1().data(),
                  qt_error_string().toLocal8Bit().data());
        return;
    }

    int sId;

    while(query.next())
    {
          sId = query.value(0).toInt() + 1;
    }

    query.prepare("INSERT INTO ActionPossibilities VALUES (:apId, :apText, 1, :locn, :apPhrase, null, 0,:pred,100,100)");


    query.bindValue(":apId",sId);
    query.bindValue(":apText",ui->APTextComboBox->currentText().section("::",1,1));
    query.bindValue(":apPhrase",ui->APPhraseComboBox->currentText().section("::",1,1));
    query.bindValue(":pred",ui->APPredComboBox->currentText().section("::",1,1));
    query.bindValue(":locn",experimentLocationString);

  //  qDebug() << sId;
  //  qDebug() << ui->APTextComboBox->currentText().section("::",1,1);
  //  qDebug() << ui->APPhraseComboBox->currentText().section("::",1,1);
  //  qDebug() << ui->APPredComboBox->currentText().section("::",1,1);

  //  qDebug()<<query.executedQuery();

    if (!query.exec())
    {

        qDebug() << query.lastQuery();

        QMessageBox msgBox;
        msgBox.setIcon(QMessageBox::Critical);

        msgBox.setText("Can't add to ActionPossibilities table - duplicate?");
        msgBox.exec();

        qCritical("Cannot add/update: %s (%s)",
                  db.lastError().text().toLatin1().data(),
                  qt_error_string().toLocal8Bit().data());

        qDebug()<<query.lastError();
        qDebug()<<query.executedQuery();

        return;
    }

    QString sIdStg;
    sIdStg.setNum(sId);
    QString msg;
    msg = "Action Possibility " + sIdStg + " Created!";
    const char* test = msg.toAscii().data();
    QMessageBox::question(this, tr("Action Possibility Creator"),
                          tr(test), QMessageBox::Ok,QMessageBox::Ok);

    fillDisplayArea();
}

void MainWindow::fillDisplayArea()
{

    ui->tableWidget->clear();

    QStringList m_TableHeader;
    m_TableHeader<<"Id"<<"Text"<<"Phrase"<<"Threshold"<<"Predicate"<<"Value";

    ui->tableWidget->setHorizontalHeaderLabels(m_TableHeader);



    QSqlQuery query;
    QString sQry;

    sQry = "SELECT  count(*) FROM ActionPossibilities A, Messages M1, Messages M2";
    sQry += " WHERE M1.messageId = ap_text";
    sQry += "  AND  M2.messageId = ap_phrase";
    sQry += "  AND   M1.languageId = " + languageId + " and  M2.languageId = " + languageId;

 //   qDebug()<<sQry;

     query = sQry;
     query.exec();

     while(query.next())
     {
           ui->tableWidget->setRowCount(query.value(0).toInt());
     }

 //   query = "SELECT * FROM ActionRules WHERE experimentalLocationId = " + locn + " and name = '" + seq + "'";

     sQry = "SELECT  apId,M1.message,M2.message,likelihood,precondid,S.value  FROM ActionPossibilities A, Messages M1, Messages M2, Sensors S";
     sQry += " WHERE M1.messageId = ap_text";
     sQry += "  AND  M2.messageId = ap_phrase";
     sQry += "  AND  M1.languageId = " + languageId + " and  M2.languageId = " + languageId;
     sQry += "  AND  S.sensorId = precondid ORDER BY apId";
 //   qDebug()<<sQry;
     query = sQry;
     query.exec();

     int row=0;

     while(query.next())
     {
  //       qDebug()<<query.value(0).toString();
        ui->tableWidget->setItem(row, 0, new QTableWidgetItem(query.value(0).toString()));
        ui->tableWidget->setItem(row, 1, new QTableWidgetItem(query.value(1).toString()));
        ui->tableWidget->setItem(row, 2, new QTableWidgetItem(query.value(2).toString()));
        ui->tableWidget->setItem(row, 3, new QTableWidgetItem(query.value(3).toString()));
        ui->tableWidget->setItem(row, 4, new QTableWidgetItem(query.value(4).toString()));
        ui->tableWidget->setItem(row, 5, new QTableWidgetItem(query.value(5).toString()));
        row++;
     }

     ui->tableWidget->resizeColumnsToContents();

     ui->APDeletePushButton->setEnabled(false);
}

void MainWindow::on_tableWidget_itemActivated(QTableWidgetItem *item)
{
    ui->APDeletePushButton->setEnabled(true);

    deleteCandidate = item->row();


}

//void MainWindow::on_tableWidget_itemSelectionChanged()
//{
//    ui->APDeletePushButton->setEnabled(false);
//}

void MainWindow::on_APDeletePushButton_clicked()
{
    int ret = QMessageBox::warning(this, tr("Action Possibility Creator"),
                                   tr("Do you really want to delete this Action Possibility?"),
                                   QMessageBox::Cancel | QMessageBox::Yes);

    if (ret == QMessageBox::Cancel)
    {
        return;
    }

    QSqlQuery query;
    query = "DELETE FROM ActionPossibilities Where apId = " + ui->tableWidget->item(deleteCandidate,0)->text();
    query.exec();
    fillDisplayArea();
}

void MainWindow::on_APRefreshPushButton_clicked()
{
        fillDisplayArea();
}

void MainWindow::on_tableWidget_itemPressed(QTableWidgetItem *item)
{
        ui->APDeletePushButton->setEnabled(true);
        deleteCandidate = item->row();
}

void MainWindow::on_tableWidget_clicked(const QModelIndex &index)
{
          ui->APDeletePushButton->setEnabled(true);
          deleteCandidate = index.row();
}

void MainWindow::on_tableWidget_doubleClicked(const QModelIndex &index)
{
              ui->APDeletePushButton->setEnabled(true);
              deleteCandidate = index.row();
}
