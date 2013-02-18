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
#include <QStringListModel>
#include <QStringList>
#include <QStandardItemModel>
#include "db/subjectmenu.h"
#include "db/locationmenu.h"
#include <QList>

#include <QSqlRelationalTableModel>
QSqlDatabase db;
bool dbOpen;

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    LocationMenu::addToComboBox(LocationMenu::User, ui->cboUserLocation);
    LocationMenu::addToComboBox(LocationMenu::Robot, ui->cboRobotLocation);
    SubjectMenu::addToComboBox(SubjectMenu::User,ui->cboUserName);
    SubjectMenu::addToComboBox(SubjectMenu::Robot,ui->cboRobotName);

    connect(ui->cboUserName, SIGNAL(currentIndexChanged(int)),this, SLOT(cboUserName_currentIndexChanged(int)));
    connect(ui->cboRobotName, SIGNAL(currentIndexChanged(int)),this, SLOT(cboRobotName_currentIndexChanged(int)));

    //set initial location without triggering location_change
    cboUserName_currentIndexChanged(ui->cboUserName->currentIndex());
    cboRobotName_currentIndexChanged(ui->cboRobotName->currentIndex());

    connect(ui->cboUserLocation, SIGNAL(currentIndexChanged(int)),this, SLOT(cboUserLocation_currentIndexChanged(int)));
    connect(ui->cboRobotLocation, SIGNAL(currentIndexChanged(int)),this, SLOT(cboRobotLocation_currentIndexChanged(int)));

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

void MainWindow::cboUserName_currentIndexChanged(int index)
{
    int userLocation = SubjectMenu::getSubjectLocation(SubjectMenu::User, ui->cboUserName->itemData(index).toInt());
    int index2 = ui->cboUserLocation->findData(userLocation);
    if (index2 != -1 ) { // -1 for not found
       ui->cboUserLocation->setCurrentIndex(index2);
    }
}

void MainWindow::cboRobotName_currentIndexChanged(int index)
{
    int robotLocation = SubjectMenu::getSubjectLocation(SubjectMenu::Robot, ui->cboRobotName->itemData(index).toInt());
    int index2 = ui->cboRobotLocation->findData(robotLocation);
    if (index2 != -1 ) { // -1 for not found
       ui->cboRobotLocation->setCurrentIndex(index2);
    }

}

void MainWindow::cboUserLocation_currentIndexChanged(int index)
{
    SubjectMenu::setSubjectLocation(SubjectMenu::User, ui->cboUserName->itemData(ui->cboUserName->currentIndex()).toInt(), ui->cboUserLocation->itemData(index).toInt());
}

void MainWindow::cboRobotLocation_currentIndexChanged(int index)
{
    SubjectMenu::setSubjectLocation(SubjectMenu::Robot, ui->cboRobotName->itemData(ui->cboRobotName->currentIndex()).toInt(), ui->cboRobotLocation->itemData(index).toInt());
}
