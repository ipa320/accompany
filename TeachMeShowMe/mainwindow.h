#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QModelIndex>
namespace Ui {
    class MainWindow;
}

class MainWindow : public QMainWindow {
    Q_OBJECT
public:
    MainWindow(QWidget *parent = 0);
    ~MainWindow();

    bool closeDownRequest;
     QString param;
    bool openDatabase();

protected:
    void changeEvent(QEvent *e);



private:
    Ui::MainWindow *ui;
;

    void initialiseGUI();

    void addToLearningList(QString learntItem);
    void checkOKVisibleFlag();
    void fillLocationWhen();
    void fillFinalView();
    void createGOTOsequence(QString seq);
    void addActionRulesRow(QString sequenceName, QString ruleText, QString actionText, int ruleCount, QString ruleActionFlag);
    void addSequenceRow(QString sequenceName, QString scenText, int ruleCount, int priority, int scheduled);
    void deleteExistingBehaviour(QString name);
    QString createResetCondition(QString seq);
    void generateForWithinSQL(QString command, QString cmd, QString sql, QString& actionText );
    QString createForWithinMsg(QString cmd, int happening, int happened, QString normal, QString normalHappining, QString normalHappened);

private slots:

    void on_bathFlushCheckBox_clicked(bool checked);
    void on_bathDoorCheckBox_clicked(bool checked);
    void on_everyNHoursSpinBox_valueChanged(int );
    void on_bathTapsCheckBox_clicked(bool checked);
    void on_doorbellCheckBox_clicked(bool checked);
    void on_fridgeCheckBox_clicked(bool checked);
    void on_microwaveCheckBox_clicked(bool checked);
    void on_smallCupboardCheckBox_clicked(bool checked);
    void on_bigCupboardCheckBox_clicked(bool checked);
    void on_sittingTableCheckBox_clicked(bool checked);
    void on_kitchenTapsCheckBox_clicked(bool checked);
    void on_kitechCupboadsCheckBox_clicked(bool checked);
    void on_hasHappendedSpinBox_valueChanged(int );
    void on_happeningSpinBox_valueChanged(int );
    void on_TVOncheckBox_clicked(bool checked);
    void on_sittingSofaCheckBox_clicked(bool checked);
    void on_alwaysCheckBox_clicked(bool checked);
    void on_allDonePushButton_clicked();
    void on_deletePushButton_clicked();
    void on_finalRememberPushButton_clicked();
    void on_learnedSoFarListWidget_doubleClicked(QModelIndex index);
    void on_everyNdaysSpinBox_valueChanged(int );
    void on_sundayCheckBox_toggled(bool checked);
    void on_saturdayCheckBox_toggled(bool checked);
    void on_fridayCheckBox_toggled(bool checked);
    void on_wednesdayCheckBox_toggled(bool checked);
    void on_ThursdayCheckBox_toggled(bool checked);
    void on_tuesdayCheckBox_toggled(bool checked);
    void on_mondayCheckBox_toggled(bool checked);
    void on_trayLearnItPushButton_clicked();
    void on_contextFinishedPushButton_clicked();
    void on_contextLearnItPushButton_clicked();
    void on_toldFinishedPushButton_clicked();
    void on_BathroomLearnItPushButton_clicked();
    void on_ElectricalLearnItPushButton_clicked();
    void on_kitchenLearnItPushButton_clicked();
    void on_livingLearnItPushButton_clicked();
    void on_diningLearnItPushButton_clicked();
    void on_houseFinishedPushButton_clicked();
    void on_timeFinishedPushButton_clicked();
    void on_locnFinishedPushButton_clicked();
    void on_locationListView_clicked(QModelIndex index);
    void on_andRobotLocnComboBox_activated(QString );
    void on_userLocnComboBox_activated(QString );
    void on_locationLearnItPushButton_clicked();
//    void on_everySoManyHoursCheckBox_toggled(bool checked);
//    void on_afterThisManyHoursCheckBox_toggled(bool checked);
//    void on_hoursDial_valueChanged(int value);
    void on_timeLearnItPushButton_clicked();
    void on_betweenCheckBox_toggled(bool checked);
    void on_atCheckBox_toggled(bool checked);
    void on_okPushButton_clicked();

    void on_timePushButton_clicked(bool checked);
    void on_locationPushButton_clicked(bool checked);
    void on_doingPushButton_clicked(bool checked);
    void on_whenSomethingPushButton_clicked(bool checked);
    void on_whenTellMePushButton_clicked(bool checked);
    void on_rememberThisPushButton_clicked();
    void on_moveKnowHowListView_clicked(QModelIndex index);
    void on_moveComboBox_activated(QString );
    void on_speakKnowHowListView_clicked(QModelIndex index);
    void on_moveLearnItPushButton_clicked();
    void on_speakLineEdit_textChanged(QString );
    void on_speakLearnItPushButton_clicked();
    void on_everythingLearnItPushButton_clicked();
    void on_alreadyKnowWidget_currentChanged(int index);
    void on_addPushButton_clicked();
    void on_behNameComboBox_activated(QString );
    void on_behNameComboBox_editTextChanged(QString );
    void on_stopLearningPushButton_clicked();
    void on_startLearningPushButton_clicked();
    void on_showMePushButton_clicked();
    void on_cancelPushButton_clicked();
    void on_teachMePushButton_clicked();


    void on_repeatFinishedPushButton_clicked();
    void on_repeatLearnItPushButton_clicked();
    void on_changePushButton_clicked();

};

#endif // MAINWINDOW_H
