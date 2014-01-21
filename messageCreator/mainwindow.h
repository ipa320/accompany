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

    bool closeDownRequest;

    void setup();
    QString lv;
    void fillLanguageCombo();
    void fillMessageCombo();

private slots:
    void on_languageComboBox_currentIndexChanged(const QString &arg1);

    void on_messageComboBox_currentIndexChanged(const QString &arg1);

    void on_changePushButton_clicked();

private:
    Ui::MainWindow *ui;
};

#endif // MAINWINDOW_H
