/****************************************************************************
** Meta object code from reading C++ file 'mainwindow.h'
**
** Created: Fri Mar 8 12:57:01 2013
**      by: The Qt Meta Object Compiler version 62 (Qt 4.6.2)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../../include/COBScheduler/mainwindow.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'mainwindow.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 62
#error "This file was generated using the moc from 4.6.2. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_MainWindow[] = {

 // content:
       4,       // revision
       0,       // classname
       0,    0, // classinfo
      15,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: signature, parameters, type, tag, flags
      12,   11,   11,   11, 0x08,
      43,   11,   11,   11, 0x08,
      89,   78,   11,   11, 0x08,
     133,   78,   11,   11, 0x08,
     183,   11,   11,   11, 0x08,
     215,   11,   11,   11, 0x08,
     246,   11,   11,   11, 0x08,
     284,   11,   11,   11, 0x08,
     321,   11,   11,   11, 0x08,
     339,   11,   11,   11, 0x08,
     352,   11,   11,   11, 0x08,
     389,   11,   11,   11, 0x08,
     420,   11,   11,   11, 0x08,
     462,  454,   11,   11, 0x08,
     500,   11,   11,   11, 0x08,

       0        // eod
};

static const char qt_meta_stringdata_MainWindow[] = {
    "MainWindow\0\0on_executePushButton_clicked()\0"
    "on_evaluateAllPushButton_clicked()\0"
    "row,column\0on_sequenceTableWidget_cellClicked(int,int)\0"
    "on_sequenceTableWidget_cellDoubleClicked(int,int)\0"
    "on_evaluatePushButton_clicked()\0"
    "on_COBTestPushButton_clicked()\0"
    "on_startSchedulerPushButton_clicked()\0"
    "on_stopSchedulerPushButton_clicked()\0"
    "doSchedulerWork()\0updateTime()\0"
    "on_initialiseAllPushButton_clicked()\0"
    "on_stopAllPushButton_clicked()\0"
    "on_recoverAllPushButton_clicked()\0"
    "checked\0on_showNonSchedcheckBox_toggled(bool)\0"
    "on_testPlannerPushButton_clicked()\0"
};

const QMetaObject MainWindow::staticMetaObject = {
    { &QMainWindow::staticMetaObject, qt_meta_stringdata_MainWindow,
      qt_meta_data_MainWindow, 0 }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &MainWindow::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *MainWindow::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *MainWindow::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_MainWindow))
        return static_cast<void*>(const_cast< MainWindow*>(this));
    return QMainWindow::qt_metacast(_clname);
}

int MainWindow::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QMainWindow::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        switch (_id) {
        case 0: on_executePushButton_clicked(); break;
        case 1: on_evaluateAllPushButton_clicked(); break;
        case 2: on_sequenceTableWidget_cellClicked((*reinterpret_cast< int(*)>(_a[1])),(*reinterpret_cast< int(*)>(_a[2]))); break;
        case 3: on_sequenceTableWidget_cellDoubleClicked((*reinterpret_cast< int(*)>(_a[1])),(*reinterpret_cast< int(*)>(_a[2]))); break;
        case 4: on_evaluatePushButton_clicked(); break;
        case 5: on_COBTestPushButton_clicked(); break;
        case 6: on_startSchedulerPushButton_clicked(); break;
        case 7: on_stopSchedulerPushButton_clicked(); break;
        case 8: doSchedulerWork(); break;
        case 9: updateTime(); break;
        case 10: on_initialiseAllPushButton_clicked(); break;
        case 11: on_stopAllPushButton_clicked(); break;
        case 12: on_recoverAllPushButton_clicked(); break;
        case 13: on_showNonSchedcheckBox_toggled((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 14: on_testPlannerPushButton_clicked(); break;
        default: ;
        }
        _id -= 15;
    }
    return _id;
}
QT_END_MOC_NAMESPACE
