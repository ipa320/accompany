#ifndef TESTSHOWME_H
#define TESTSHOWME_H

#include <QDialog>

namespace Ui {
class testShowMe;
}

class testShowMe : public QDialog
{
    Q_OBJECT
    
public:
    explicit testShowMe(QWidget *parent = 0);
    ~testShowMe();

    void setTiming(int nTiming);
    
private:
    Ui::testShowMe *ui;

    void createCase(QString classification);
    float classify(QString item);
    bool updateContextSensor(QString context, bool outcome);

private slots:

    void doClassification();


};

#endif // TESTSHOWME_H
