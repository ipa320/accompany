#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QFile>
#include <QDebug>
#include <QInputDialog>
#include <QMessageBox>
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

    bool openDatabase();


    bool closeDownRequest;

    QString param;

private:
    Ui::MainWindow *ui;

    void createCase(QString classification);
    float classify(QString item);
    bool updateContextSensor(QString context, bool outcome);

private slots:

    void doClassification();


    void on_stopPushButton_clicked();
    void on_startPushButton_clicked();
};

#endif // MAINWINDOW_H
