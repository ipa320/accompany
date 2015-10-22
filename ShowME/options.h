#ifndef OPTIONS_H
#define OPTIONS_H

#include <QDialog>
#include <QDebug>

namespace Ui {
class options;
}

class options : public QDialog
{
    Q_OBJECT
    
public:
    explicit options(QWidget *parent = 0);
    ~options();

    double getTiming();

    void setTiming(double v);

    void setNegativeExampleState(bool swc);

    bool getNegativeExampleState();

    
private slots:
    void on_buttonBox_accepted();
    void on_buttonBox_rejected();
    void on_clearPushButton_clicked();

private:
    Ui::options *ui;
};

#endif // OPTIONS_H
