#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QtSql>

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <cob_script_server/ScriptAction.h>

typedef actionlib::SimpleActionClient<cob_script_server::ScriptAction> Client;

namespace Ui {
    class MainWindow;
}

class MainWindow  : public QMainWindow {
    Q_OBJECT
public:
    MainWindow(int argc, char** argv, QWidget *parent = 0);
    ~MainWindow();
    bool evaluateRules(QString sequenceName, bool display);
    int  executeSequence(QString sequenceName, bool display);
    void logMessage(QString msg);
    bool fillSequenceTable();
    void initialise_COB_components();
    void recover_COB_components();
    void stop_COB_Components();

    void stopSequence(QString sequenceName);

    void runSequence(QString sequenceName, int priority, QString CanInterrupt, int row);
    void checkExecutionResult();
    bool closeDownRequest;
    void COB_component(QString component, QString action);
    bool openDatabase(QString dbName, QString host, QString user, QString pw, QSqlDatabase& db);
    bool openAllDatabaseConnections(QString host, QString user, QString pw);
    void closeAllDatabaseConnections();
    void planNavigation(QString destination);
    int  sendScriptServerMsg();


protected:
    void changeEvent(QEvent *e);

private:
    Ui::MainWindow *ui;

    QTimer timer;

    QTimer schedTimer;



private slots:

    void on_executePushButton_clicked();
    void on_evaluateAllPushButton_clicked();
    void on_sequenceTableWidget_cellClicked(int row, int column);
    void on_sequenceTableWidget_cellDoubleClicked(int row, int column);
    void on_evaluatePushButton_clicked();
    void on_COBTestPushButton_clicked();
    void on_startSchedulerPushButton_clicked();
    void on_stopSchedulerPushButton_clicked();
    void doSchedulerWork();
    void updateTime();
    void on_initialiseAllPushButton_clicked();
    void on_stopAllPushButton_clicked();
    void on_recoverAllPushButton_clicked();
    void on_showNonSchedcheckBox_toggled(bool checked);
    void on_testPlannerPushButton_clicked();


};

#endif // MAINWINDOW_H
