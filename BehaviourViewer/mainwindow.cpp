#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QInputDialog>
#include <QDebug>
#include <QMessageBox>

QSqlDatabase db;
bool dbOpen;
int experimentLocation;

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
       experimentLocation = query.value(0).toInt();
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


    fillScenarioCombo();
    fillSequenceCombo();

    QString top;
    top = ui->comboBox->currentText();

    fillDisplayArea(top);

 }


void MainWindow::fillSequenceCombo()
{
    QString seqQuery;
    QSqlQuery query;
    QString locn;
    locn.setNum(experimentLocation);

    seqQuery = "SELECT * FROM Sequences WHERE experimentalLocationId = " + locn;

    if (ui->checkBox_2->isChecked())
    {
        seqQuery += " and schedulable = 1";
    }


    if (ui->scenComboBox->currentText() != "")
    {
        seqQuery += " and scenario = '" + ui->scenComboBox->currentText();
    }

    if (ui->checkBox->isChecked())
    {
        seqQuery += "' order by priority DESC";
    }
    else
    {
        seqQuery += "' order by name";
    }


    query = seqQuery;


    ui->comboBox->clear();

    while(query.next())
    {   qDebug()<< query.value(0).toString();
        ui->comboBox->addItem(query.value(0).toString());
    }

    ui->comboBox->clearEditText();


}


void MainWindow::fillScenarioCombo()
{
    QString senQuery;
    QSqlQuery query;


    senQuery = "SELECT scenario FROM ScenarioType order by scenario";

    query = senQuery;


    ui->scenComboBox->clear();

    while(query.next())
    {   qDebug()<< query.value(0).toString();
        ui->scenComboBox->addItem(query.value(0).toString());
    }

    ui->scenComboBox->clearEditText();


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

void MainWindow::on_comboBox_currentIndexChanged(QString seq)
{
    fillDisplayArea(seq);
}

void MainWindow::fillDisplayArea(QString seq)
{

    ui->tableWidget->clear();

    QStringList m_TableHeader;
    m_TableHeader<<"Rule"<<"Action"<<"SQL Rule"<<"Script Server Action";

    ui->tableWidget->setHorizontalHeaderLabels(m_TableHeader);

    QString locn;
    locn.setNum(experimentLocation);

    QSqlQuery query;

    query = "SELECT count(*) FROM ActionRules WHERE experimentalLocationId = " + locn + " and name = '" + seq + "'";

     query.exec();

     while(query.next())
     {
           ui->tableWidget->setRowCount(query.value(0).toInt());
     }

    query = "SELECT * FROM ActionRules WHERE experimentalLocationId = " + locn + " and name = '" + seq + "'";

     query.exec();

     int row=0;

     while(query.next())
     {
        if (query.value(2).toString() == "R")
        {
          ui->tableWidget->setItem(row, 0, new QTableWidgetItem(query.value(5).toString()));
          ui->tableWidget->setItem(row, 2, new QTableWidgetItem(query.value(6).toString()));
          ui->tableWidget->setItem(row, 1, new QTableWidgetItem("     "));
          ui->tableWidget->setItem(row, 3, new QTableWidgetItem("     "));
        }
        else
        {
          ui->tableWidget->setItem(row, 0, new QTableWidgetItem("     "));
          ui->tableWidget->setItem(row, 2, new QTableWidgetItem("     "));
          ui->tableWidget->setItem(row, 1, new QTableWidgetItem(query.value(5).toString()));
          ui->tableWidget->setItem(row, 3, new QTableWidgetItem(query.value(7).toString()));
        }

        row++;

     }

     ui->tableWidget->resizeColumnsToContents();
}

void MainWindow::on_checkBox_toggled(bool checked)
{
    fillSequenceCombo();
}

void MainWindow::on_checkBox_2_toggled(bool checked)
{
       fillSequenceCombo();
}

void MainWindow::on_pushButton_clicked()
{

    QString Seq = ui->comboBox->currentText();


    QPrinter printer;

    QPrintDialog *dialog = new QPrintDialog(&printer, this);
    dialog->setWindowTitle(tr("Print Document"));

  //  if (editor->textCursor().hasSelection())
  //      dialog->addEnabledOption(QAbstractPrintDialog::PrintSelection);
    if (dialog->exec() != QDialog::Accepted)
        return;

    QPainter painter;
    painter.begin(&printer);

    QString text;

    int rows = ui->tableWidget->rowCount();

    int n = 100;

    painter.drawText(100, n, 500, 500, Qt::AlignLeft|Qt::AlignTop, Seq);

    n+=60;


    for (int i =0; i<rows; i++)
    {
        text = ui->tableWidget->item(i,0)->text() + " "
             + ui->tableWidget->item(i,1)->text();


        painter.drawText(100, n, 500, 500, Qt::AlignLeft|Qt::AlignTop, text);

        n+=20;

     }

    for (int i =0; i<4; i++)
    {
        text = " ";


        painter.drawText(100, n, 500, 500, Qt::AlignLeft|Qt::AlignTop, text);

        n+=20;

     }

    for (int i =0; i<rows; i++)
    {
        text = ui->tableWidget->item(i,2)->text() + " "
             + ui->tableWidget->item(i,3)->text();

        qDebug()<<text;
        painter.drawText(100, n, 500, 500, Qt::AlignLeft|Qt::AlignTop, text);

        n+=20;

     }
    painter.end();


}

void MainWindow::on_scenComboBox_currentIndexChanged(const QString &arg1)
{
        fillSequenceCombo();
}
