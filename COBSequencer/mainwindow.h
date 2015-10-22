#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QtSql>

namespace Ui {
    class MainWindow;
}

class MainWindow : public QMainWindow {
    Q_OBJECT
public:
    MainWindow(QWidget *parent = 0);
    ~MainWindow();

    bool closeDownRequest;
    QString lv;

    void setup();





protected:
    void changeEvent(QEvent *e);

private:
    Ui::MainWindow *ui;

    QTimer timer;

    void fillActionRuleTable(QString seqName);

    void updateSequenceTable();

    void fillRuleActionTable(QString name, int Id, QString type, bool checkBox,
                                            int spinBox, bool ANDRadio , bool ORRadio);
    void resetGui();

    void updateActionDB(QString effector, QString sequenceName, QString actiontext, QString action);



private slots:

    void on_apCheckBox_toggled(bool checked);
    void on_armCheckBox_toggled(bool checked);
    void on_HUYTlivingSofa3CheckBox_toggled(bool checked);
    void on_HUYTlivingSofa2CheckBox_toggled(bool checked);
    void on_robotExpressionCheckBox_toggled(bool checked);
    void on_ZUYDFridgeCheckBox_toggled(bool checked);
    void on_ZUYDDoorbellCheckBox_toggled(bool checked);
    void on_cupLevelCheckBox_toggled(bool checked);
    void on_HUYTlivingSofa1CheckBox_toggled(bool checked);
    void on_Goal1ComboBox_currentIndexChanged(QString );
    void on_seqTypeComboBox_currentIndexChanged(int index);
    void on_seqDescLineEdit_editingFinished();
    void on_condAddRuleButton_clicked();
    void on_robotEyesCheckBox_toggled(bool checked);
    void on_robotGoal1CheckBox_toggled(bool checked);
    void on_Goal1CheckBox_toggled(bool checked);

    void updateTime();

    void on_GUI4CheckBox_clicked(bool checked);
    void on_GUI3CheckBox_clicked(bool checked);
    void on_GUI2CheckBox_clicked(bool checked);
    void on_GUI1CheckBox_clicked(bool checked);
    void on_robotGUI_toggled(bool checked);
    void on_moveRobotComboBox_currentIndexChanged(QString );
    void on_scheduleCheckBox_clicked();
    void on_SmallCupboardDoorTopCheckBox_toggled(bool checked);
    void on_SmallCupboardDoorMiddleCheckBox_toggled(bool checked);
    void on_SmallCupboardDrawBottomCheckBox_toggled(bool checked);
    void on_SmallCupboardDooRightCheckBox_toggled(bool checked);
    void on_SmallCupboardDoorLeftCheckBox_toggled(bool checked);
    void on_BigCupboardDoorTopCheckBox_toggled(bool checked);
    void on_BigCupboardDoorBottomCheckBox_toggled(bool checked);

    void on_robotDelayCheckBox_toggled(bool checked);
    void on_actionSequenceCheckBox_toggled(bool checked);
    void on_robotSpeakCheckBox_toggled(bool checked);
    void on_robotLightCheckBox_toggled(bool checked);
    void on_torsoLookLeftRadioButton_toggled(bool checked);
    void on_torsoLookRightRadioButton_toggled(bool checked);
    void on_torsoLookForwardRadioButton_toggled(bool checked);
    void on_torsoLookBackRadioButton_toggled(bool checked);
    void on_torsoHomeCheckBox_toggled(bool checked);
    void on_robotTorsoCheckBox_toggled(bool checked);
    void on_lastActiveCheckBox_toggled(bool checked);
    void on_sensorActiveCheckBox_toggled(bool checked);
    void on_timeAtRadioButton_clicked();
    void on_timeBetweenRadioButton_clicked();
    void on_timeCheckBox_toggled(bool checked);
    void on_doorbellCheckBox_toggled(bool checked);
    void on_dishwasherCheckBox_toggled(bool checked);
    void on_toasterCheckBox_toggled(bool checked);
    void on_kettleCheckBox_toggled(bool checked);
    void on_fridgeCheckBox_toggled(bool checked);
    void on_cookerCheckBox_toggled(bool checked);
    void on_microwaveCheckBox_toggled(bool checked);
    void on_KitchenFloorDoorRightCheckBox_toggled(bool checked);
    void on_KitchenFloorDoorMiddleCheckBox_toggled(bool checked);
    void on_KitchenFloorDoorLeftCheckBox_toggled(bool checked);
    void on_KitchenFloorDrawerRightCheckBox_toggled(bool checked);
    void on_KitchenFloorDrawerMiddleCheckBox_toggled(bool checked);
    void on_KitchenCeilDoorRightCheckBox_toggled(bool checked);
    void on_KitchenCeilDoorMiddleCheckBox_toggled(bool checked);
    void on_KitchenCeilDoorLeftCheckBox_toggled(bool checked);
    void on_ColdTapKitchenCheckBox_toggled(bool checked);
    void on_HotTapKitchenCheckBox_toggled(bool checked);
    void on_addActionButton_clicked();
    void on_robotTrayCheckBox_toggled(bool checked);
    void on_moveRobotCheckBox_toggled(bool checked);
    void on_TVcheckBox_toggled(bool checked);
    void on_livingSofa5CheckBox_toggled(bool checked);
    void on_livingSofa4CheckBox_toggled(bool checked);
    void on_livingSofa3CheckBox_toggled(bool checked);
    void on_livingSofa2CheckBox_toggled(bool checked);
    void on_livingSofa1CheckBox_toggled(bool checked);
    void on_prioritySpinBox_editingFinished();
    void on_InterruptcheckBox_clicked();
    void on_delRuleButton_clicked();
    void on_ruleListView_clicked(QModelIndex index);
    void on_addRuleButton_clicked();
    void on_seqDelButton_clicked();
    void on_SeqComboBox_activated(QString );
    void on_seqAddButton_clicked();
    void on_SeqComboBox_editTextChanged(QString );
    void on_diningSofa2CheckBox_toggled(bool checked);
    void on_diningSofa1CheckBox_toggled(bool checked);
    void on_selectSensorsButton_clicked();
    void on_IgnoreSensorsButton_clicked();

    void on_robotLocationAnyButton_toggled(bool checked);
    void on_AnyUserLocationButton_toggled(bool checked);
    void on_AnyUserLocationButton_clicked();

    void on_robotLocationSpecButton_clicked();
    void on_robotLocationAnyButton_clicked();
    void on_specUserLocationButton_clicked();
    void on_playSoundCheckBox_toggled(bool checked);
    void on_robotSaveMemoryCheckBox_toggled(bool checked);
    void on_UHcupLevelCheckBox_toggled(bool checked);
    void on_reOrderButton_clicked();
    void on_updateOrderButton_clicked();
    void on_dropEvent(QDropEvent * e);


    void on_E125_SS1_CheckBox_toggled(bool checked);
    void on_E125_SS2_CheckBox_toggled(bool checked);
    void on_E125_LS1_CheckBox_toggled(bool checked);
    void on_E125_LS2_CheckBox_toggled(bool checked);
    void on_LRDCheckBox_toggled(bool checked);
    void on_BRDCheckBox_toggled(bool checked);
};

#endif // MAINWINDOW_H
