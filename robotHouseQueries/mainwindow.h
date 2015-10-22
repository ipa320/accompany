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
    MainWindow(QWidget *parent = 0);
    ~MainWindow();
    QString lv;

protected:
    void changeEvent(QEvent *e);

private:
    Ui::MainWindow *ui;

private slots:
    void on_refreshButton_stateChanged(int );
    void on_sensorCheckBox_stateChanged(int );
    void on_sensorDefaultsButton_clicked();
    void on_emptyLogButton_clicked();
    void on_sensorLogButton_clicked();
    void on_robotButton_clicked();
    void on_peopleButton_clicked();
    void on_sensorsButton_clicked();
    void on_objectsButton_clicked();
    void on_locationsButton_clicked();
    void on_openDBButton_clicked();
    void updateSensors();
    void updateSensorLog();
};




#endif // MAINWINDOW_H
