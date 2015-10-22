#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QFile>
#include <QDebug>
#include <QInputDialog>
#include <QMessageBox>
#include <QtSql>

#include "options.h"
#include "testshowme.h"


namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT
    
public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

    bool openDatabase();

    bool closeDownRequest;

    QString param;
    
private slots:
    void on_showMePushButton_clicked();




    void on_showMeForgetButton_clicked();

    void on_showMeStopPushButton_clicked();


    void on_showMeComboBox_currentIndexChanged(const QString &arg1);

    bool checkTaskExists(QString task);

    void on_showMeComboBox_2_currentIndexChanged(const QString &arg1);

    void on_showMePushButton_2_clicked();
    void blink();


    void on_showMeStopPushButton_2_clicked();

    void on_showMeForgetButton_2_clicked();

    void on_showMeComboBox_3_currentIndexChanged(const QString &arg1);

    void on_showMePushButton_3_clicked();


    void on_showMeStopPushButton_3_clicked();

    void on_showMeForgetButton_3_clicked();

//    void on_pushButton_clicked();

    void closeEvent (QCloseEvent *event);

    void createStateVector(QString classification);

    void on_timerPushButton_clicked();

    void on_testPushButton_clicked();

    void updateDataSets();

private:
    Ui::MainWindow *ui;

    void initialiseGUI();

    bool deleteActivity(QString activity);

};

#endif // MAINWINDOW_H
