/****************************************************************************
** Meta object code from reading C++ file 'mainwindow.h'
**
** Created: Mon Sep 16 09:54:22 2013
**      by: The Qt Meta Object Compiler version 63 (Qt 4.8.1)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "mainwindow.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'mainwindow.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 63
#error "This file was generated using the moc from 4.8.1. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_MainWindow[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
      17,   14, // methods
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
     360,  352,   11,   11, 0x08,
     398,   11,   11,   11, 0x08,
     433,   11,   11,   11, 0x08,
     468,   11,   11,   11, 0x08,
     497,   11,   11,   11, 0x08,
     526,   11,   11,   11, 0x08,
     555,   11,   11,   11, 0x08,

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
    "doSchedulerWork()\0updateTime()\0checked\0"
    "on_showNonSchedcheckBox_toggled(bool)\0"
    "on_testPlannerPushButton_clicked()\0"
    "on_enableDebugPushButton_clicked()\0"
    "on_GUIradioButton1_clicked()\0"
    "on_GUIradioButton2_clicked()\0"
    "on_GUIradioButton3_clicked()\0"
    "on_GUIradioButton4_clicked()\0"
};

void MainWindow::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        MainWindow *_t = static_cast<MainWindow *>(_o);
        switch (_id) {
        case 0: _t->on_executePushButton_clicked(); break;
        case 1: _t->on_evaluateAllPushButton_clicked(); break;
        case 2: _t->on_sequenceTableWidget_cellClicked((*reinterpret_cast< int(*)>(_a[1])),(*reinterpret_cast< int(*)>(_a[2]))); break;
        case 3: _t->on_sequenceTableWidget_cellDoubleClicked((*reinterpret_cast< int(*)>(_a[1])),(*reinterpret_cast< int(*)>(_a[2]))); break;
        case 4: _t->on_evaluatePushButton_clicked(); break;
        case 5: _t->on_COBTestPushButton_clicked(); break;
        case 6: _t->on_startSchedulerPushButton_clicked(); break;
        case 7: _t->on_stopSchedulerPushButton_clicked(); break;
        case 8: _t->doSchedulerWork(); break;
        case 9: _t->updateTime(); break;
        case 10: _t->on_showNonSchedcheckBox_toggled((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 11: _t->on_testPlannerPushButton_clicked(); break;
        case 12: _t->on_enableDebugPushButton_clicked(); break;
        case 13: _t->on_GUIradioButton1_clicked(); break;
        case 14: _t->on_GUIradioButton2_clicked(); break;
        case 15: _t->on_GUIradioButton3_clicked(); break;
        case 16: _t->on_GUIradioButton4_clicked(); break;
        default: ;
        }
    }
}

const QMetaObjectExtraData MainWindow::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject MainWindow::staticMetaObject = {
    { &QMainWindow::staticMetaObject, qt_meta_stringdata_MainWindow,
      qt_meta_data_MainWindow, &staticMetaObjectExtraData }
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
        if (_id < 17)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 17;
    }
    return _id;
}
QT_END_MOC_NAMESPACE
