#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QtGui>
#include "locn.h"
#include <QtSql>
#include "newlocation.h"


namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

    QGraphicsScene* scene;
    locn* txt;

    bool    closeDownRequest;


private slots:
    void on_refreshButton_clicked();

    void on_printButton_clicked();

    void on_updateButton_clicked();

    void on_checkBox_clicked(bool checked);

    void on_checkBox_2_clicked(bool checked);

    void on_newLocnPushButton_clicked();

    void on_zoomoutPushButton_clicked();

    void on_zoomInPushButton_clicked();


    void on_voronoiPushButton_clicked();

    void on_l0checkBox_clicked(bool checked);

private:
    Ui::MainWindow *ui;

    void openDB();
    void init();
    void labelX(qreal x);
    void labelY(qreal y);

    bool orientationFlag;

    QString host, user, pw, dBase;
    QSqlDatabase db;
    bool dbOpen;
    QString experimentLocation;
    QString startLocation;
    QString endLocation;
    QString locationName;

    qreal sceneWidth;
    qreal sceneHeight;

    QPrinter* printer;

    qreal convertWall(qreal x, qreal maxXwall, qreal sceneSize);

    QGraphicsItem * vLines[100];
    int numVLines;

    void addLocnToScene(QString locnTxt,qreal x, qreal y, qreal orientation);


};

#endif // MAINWINDOW_H
