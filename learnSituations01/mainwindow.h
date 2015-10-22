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
    QString lv;
    bool closeDownRequest;

private slots:

    void on_behLogPushButton_clicked();

    void on_ruleSetPushButton_clicked();

    bool trainingLogInsert(QDateTime tDate, QString txt, QString trainingNoStr);

    void on_ruleStatePushButton_clicked();

    void on_classUpdatePushButton_clicked();

    void on_namesPushButton_clicked();

private:
    Ui::MainWindow *ui;
};

#endif // MAINWINDOW_H
