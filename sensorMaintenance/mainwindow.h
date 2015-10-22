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
    bool closeDownRequest;
    void fillSensorComboBox();
    void clearForm();
    void fillRuleComboBox(QString rule);
    void setRuleCombo();
    
private slots:
    void on_sensorComboBox_activated(const QString &arg1);

    void on_sensorComboBox_editTextChanged(const QString &arg1);
    void fillLocationComboBox(int locnId);
    void fillAccessPointComboBox(int accessPoint);
    void fillSensorTypeComboBox(int sensorType);
    bool updateSensor();

    void on_changePushButton_clicked();

    void on_delPushButton_clicked();



    void on_pushButton_clicked();

    void on_addPushButton_clicked();

private:
    Ui::MainWindow *ui;
};

#endif // MAINWINDOW_H
