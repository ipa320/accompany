#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QtSql>
#include <QTableWidgetItem>

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT
    
public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

    bool closeDownRequest;

    void setup();
    QString lv;
    void fillAPTextCombo();
    void fillAPPredicates();
    void fillDisplayArea();
    
private slots:

    void on_clearAPPushButton_clicked();

    void on_APCreatePushButton_clicked();

    void on_tableWidget_itemActivated(QTableWidgetItem *item);

 //   void on_tableWidget_itemSelectionChanged();

    void on_APDeletePushButton_clicked();

    void on_APRefreshPushButton_clicked();

    void on_tableWidget_itemPressed(QTableWidgetItem *item);

    void on_tableWidget_clicked(const QModelIndex &index);

    void on_tableWidget_doubleClicked(const QModelIndex &index);

private:
    Ui::MainWindow *ui;
};

#endif // MAINWINDOW_H
