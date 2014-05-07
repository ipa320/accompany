#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QtSql>
#include "robot.h"
#include "history.h"
#include <iostream>
#include <sstream>
#include <Python.h>

namespace Ui {
class MainWindow;
}

class MainWindow: public QMainWindow {
	Q_OBJECT
public:
	MainWindow(int argc, char** argv, QWidget *parent = 0);
	~MainWindow();
	bool evaluateRules(QString sequenceName, bool display);
    bool evaluateResources(QString sequenceName);
	int executeSequence(QString sequenceName, bool display);
	void logMessage(QString msg);
    bool fillSequenceTable(QString scenario);

	void stopSequence(QString sequenceName);

	void runSequence(QString sequenceName, int priority, QString CanInterrupt, int row);
	void checkExecutionResult();
	bool closeDownRequest;
	bool stopExecution;
	void COB_component(QString component, QString action);
    bool openDatabase(QString dbName, QString host, QString user, QString pw, QString dbase, QSqlDatabase& db);
    bool openAllDatabaseConnections(QString host, QString user, QString pw, QString dbase);
	void closeAllDatabaseConnections();
	void planNavigation(QString destination);
	int sendScriptServerMsg();
	void updateGUI(int option);


protected:
	void changeEvent(QEvent *e);

private:
	Ui::MainWindow *ui;
	QTimer timer;
	QTimer schedTimer;
	void checkStopExecution();

public slots:
    int  retryMessage(QString msg);

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

	void on_showNonSchedcheckBox_toggled(bool checked);
	void on_testPlannerPushButton_clicked();
	void on_enableDebugPushButton_clicked();
	void on_GUIradioButton1_clicked();
	void on_GUIradioButton2_clicked();
	void on_GUIradioButton3_clicked();
	void on_GUIradioButton4_clicked();
    void on_speedSpinBox_valueChanged(int arg1);
};

#endif // MAINWINDOW_H
