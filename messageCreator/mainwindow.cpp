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

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::setup()
{

    bool ok;
    QString host, user, pw, dBase;

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

    dBase = QInputDialog::getText ( this, "Accompany DB", "Database:",QLineEdit::Normal,
                                   "", &ok);
    if (!ok)
    {
       closeDownRequest = true;
       return;
    };

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

    if (dBase=="")  dBase = "Accompany";


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

    fillLanguageCombo();
    fillMessageCombo();

}

void MainWindow::fillLanguageCombo()
{
    QString seqQuery;
    QSqlQuery query;

    seqQuery = "SELECT languageID,language FROM Language";

    query = seqQuery;

    ui->languageComboBox->clear();

    while(query.next())
    {
        ui->languageComboBox->addItem("::" + query.value(0).toString() + ":: " + query.value(1).toString());
    }
}

void MainWindow::fillMessageCombo()
{
    QString seqQuery;
    QSqlQuery query;

    seqQuery = "SELECT messageId,message FROM Accompany.Messages where languageId = " + ui->languageComboBox->currentText().section("::",1,1) + " order by messageId";

    query = seqQuery;

    ui->messageComboBox->clear();
    ui->messageComboBox->addItem("Create new message");
    while(query.next())
    {
        ui->messageComboBox->addItem("::" + query.value(0).toString() + ":: " + query.value(1).toString());
    }
}

void MainWindow::on_languageComboBox_currentIndexChanged(const QString &arg1)
{
    fillMessageCombo();
}

void MainWindow::on_messageComboBox_currentIndexChanged(const QString &arg1)
{
    ui->messageLineEdit->setText(arg1.section("::",2,2));
}

void MainWindow::on_changePushButton_clicked()
{
    if (ui->messageLineEdit->text() == "")
    {
        QMessageBox::question(this, tr("Message Creator"),
                              tr("You really need to enter a message!!"),
                                            QMessageBox::Ok,QMessageBox::Ok);
        return;

    }
    if (ui->messageComboBox->currentIndex() == 0)
    {

        QSqlQuery query;

        query.prepare("SELECT MAX(messageId) FROM Messages where languageId = " + ui->languageComboBox->currentText().section("::",1,1));

        if (!query.exec())
        {
            qDebug() << query.lastQuery();

            QMessageBox msgBox;
            msgBox.setIcon(QMessageBox::Critical);

            msgBox.setText("Can't select from Messages table?");
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

        query.prepare("INSERT INTO Messages VALUES (:messId, :langId, :msg, 0)");


        query.bindValue(":messId",sId);
        query.bindValue(":langId",ui->languageComboBox->currentText().section("::",1,1));
        query.bindValue(":msg",ui->messageLineEdit->text());

        qDebug()<<query.executedQuery();

        if (!query.exec())
        {

            qDebug() << query.lastQuery();

            QMessageBox msgBox;
            msgBox.setIcon(QMessageBox::Critical);

            msgBox.setText("Can't add to Messages table - duplicate?");
            msgBox.exec();

            qCritical("Cannot add/update: %s (%s)",
                      db.lastError().text().toLatin1().data(),
                      qt_error_string().toLocal8Bit().data());
            return;
        }

        QString sIdStg;
        sIdStg.setNum(sId);
        QString msg;
        msg = "Message " + sIdStg + " Created!";
        const char* test = msg.toAscii().data();
        QMessageBox::question(this, tr("Messages Creator"),
                              tr(test),
                                            QMessageBox::Ok,QMessageBox::Ok);

        fillMessageCombo();

    }
    else
    {


        QSqlQuery query;

        query.prepare("UPDATE Messages SET message = :msg WHERE messageId = :messId and languageId = :langId");

        query.bindValue(":messId",ui->messageComboBox->currentText().section("::",1,1));
        query.bindValue(":langId",ui->languageComboBox->currentText().section("::",1,1));
        query.bindValue(":msg",ui->messageLineEdit->text());



        if (!query.exec())
        {

            qDebug() << query.lastQuery();

            QMessageBox msgBox;
            msgBox.setIcon(QMessageBox::Critical);

            msgBox.setText("Can't add to Messages table - duplicate?");
            msgBox.exec();

            qCritical("Cannot add/update: %s (%s)",
                      db.lastError().text().toLatin1().data(),
                      qt_error_string().toLocal8Bit().data());
            return;
        }


        QString msg;
        msg = "Message " + ui->messageComboBox->currentText().section("::",1,1) + " Updated!";
        const char* test = msg.toAscii().data();
        QMessageBox::question(this, tr("Messages Creator"),
                              tr(test),
                                            QMessageBox::Ok,QMessageBox::Ok);

        fillMessageCombo();
    }

}
