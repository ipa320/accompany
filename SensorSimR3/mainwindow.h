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

protected:
    void changeEvent(QEvent *e);

private:
    Ui::MainWindow *ui;

private slots:
    void cboUserName_currentIndexChanged(int index);
    void cboUserLocation_currentIndexChanged(int index);
    void cboRobotName_currentIndexChanged(int index);
    void cboRobotLocation_currentIndexChanged(int index);
};

#endif // MAINWINDOW_H
