#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QtSql>
#include <QPrinter>
#include <QPrintDialog>
#include <QPainter>

namespace Ui {
    class MainWindow;
}

class MainWindow : public QMainWindow {
    Q_OBJECT
public:
    MainWindow(QWidget *parent = 0);
    ~MainWindow();

    bool closeDownRequest;
    void fillDisplayArea(QString seq);
    void setup();
    QString lv;
    void fillSequenceCombo();

protected:
    void changeEvent(QEvent *e);

private:
    Ui::MainWindow *ui;

private slots:
    void on_checkBox_2_toggled(bool checked);
    void on_checkBox_toggled(bool checked);
    void on_comboBox_currentIndexChanged(QString );
    void on_pushButton_clicked();
    void fillScenarioCombo();
    void on_scenComboBox_currentIndexChanged(const QString &arg1);
};

#endif // MAINWINDOW_H
