#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QtSql>
namespace Ui {
    class MainWindow;
}

class MainWindow : public QMainWindow {
    Q_OBJECT
public:
    MainWindow(QWidget *parent = 0);
    ~MainWindow();
    QString lv;
    bool closeDownRequest;
    void openbDB();
    void setup();

protected:
    void changeEvent(QEvent *e);

private:
    Ui::MainWindow *ui;
    void updateSensorLog(int sensor, int value, QString stat);
    void getLocations();


private slots:


private slots:
    void on_userLocationComboBox_currentIndexChanged(QString );
    void on_robotLocationComboBox_currentIndexChanged(QString );
    void on_trayCheckBox_toggled(bool checked);
    void on_sofa3CheckBox_toggled(bool checked);
    void on_sofa2CheckBox_toggled(bool checked);
    void on_pushButton_clicked();
    void on_fridgeCheckBox_toggled(bool checked);
    void on_sofa1CheckBox_toggled(bool checked);
    void on_pushButton_2_clicked();
    void on_cupCheckBox_clicked(bool checked);
    void on_sofa4CheckBox_clicked(bool checked);
    void on_sofa5CheckBox_clicked(bool checked);
    void on_TVCheckBox_clicked(bool checked);
    void on_trayStatusCheckBox_clicked(bool checked);
    void on_kettleCheckBox_clicked(bool checked);
    void on_microCheckBox_clicked(bool checked);
    void on_toasterCheckBox_clicked(bool checked);
};

#endif // MAINWINDOW_H
