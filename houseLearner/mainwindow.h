#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QtSql>

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT
    
public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

    void setup();
    void  fillSensorStateTable();
    void  clearSensorStateTable();
    bool  checkDates();
    bool  updateTempTable();
    void insertNewRow(QDateTime newDate,QString sensorId, QString sensorValue, QString trainNo);
    void readRow();

    QString lv;
    bool closeDownRequest;
    
private slots:
    void on_clearPushButton_clicked();

    void on_fillPushButton_clicked();

    void on_inflatePushButton_clicked();

    void on_checkBox_toggled(bool checked);

    void on_checkBox_2_clicked(bool checked);

private:
    Ui::MainWindow *ui;
};

#endif // MAINWINDOW_H
