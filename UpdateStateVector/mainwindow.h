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
    void on_startPushButton_clicked();

    void on_stopPushButton_clicked();

    void createStateVector();

private:
    Ui::MainWindow *ui;




};

#endif // MAINWINDOW_H
