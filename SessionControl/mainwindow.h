#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QInputDialog>
#include <QtSql>
#include <QMessageBox>

namespace Ui {
    class MainWindow;
}

class MainWindow : public QMainWindow {
    Q_OBJECT
public:
    MainWindow(QWidget *parent = 0);
    ~MainWindow();

    bool closeDownRequest;
    void setup();
    QString lv;


private slots:
    void on_hourSpinBox_valueChanged(int );
    void on_minuteSpinBox_valueChanged(int );
    void on_userComboBox_currentIndexChanged(QString );
    void on_locationComboBox_currentIndexChanged(int index);
    void updateTime();

protected:
    void changeEvent(QEvent *e);

private:
    Ui::MainWindow *ui;


    QTimer timer;


};

#endif // MAINWINDOW_H
