/****************************************************************************
** Meta object code from reading C++ file 'mainwindow.h'
**
** Created: Tue Aug 19 14:14:13 2014
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
      19,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: signature, parameters, type, tag, flags
      20,   16,   12,   11, 0x0a,
      42,   11,   11,   11, 0x08,
      73,   11,   11,   11, 0x08,
     119,  108,   11,   11, 0x08,
     163,  108,   11,   11, 0x08,
     213,   11,   11,   11, 0x08,
     245,   11,   11,   11, 0x08,
     276,   11,   11,   11, 0x08,
     314,   11,   11,   11, 0x08,
     351,   11,   11,   11, 0x08,
     369,   11,   11,   11, 0x08,
     390,  382,   11,   11, 0x08,
     428,   11,   11,   11, 0x08,
     463,   11,   11,   11, 0x08,
     498,   11,   11,   11, 0x08,
     527,   11,   11,   11, 0x08,
     556,   11,   11,   11, 0x08,
     585,   11,   11,   11, 0x08,
     619,  614,   11,   11, 0x08,

       0        // eod
};

static const char qt_meta_stringdata_MainWindow[] = {
    "MainWindow\0\0int\0msg\0retryMessage(QString)\0"
    "on_executePushButton_clicked()\0"
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
    "on_GUIradioButton4_clicked()\0arg1\0"
    "on_speedSpinBox_valueChanged(int)\0"
};

void MainWindow::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        MainWindow *_t = static_cast<MainWindow *>(_o);
        switch (_id) {
        case 0: { int _r = _t->retryMessage((*reinterpret_cast< QString(*)>(_a[1])));
            if (_a[0]) *reinterpret_cast< int*>(_a[0]) = _r; }  break;
        case 1: _t->on_executePushButton_clicked(); break;
        case 2: _t->on_evaluateAllPushButton_clicked(); break;
        case 3: _t->on_sequenceTableWidget_cellClicked((*reinterpret_cast< int(*)>(_a[1])),(*reinterpret_cast< int(*)>(_a[2]))); break;
        case 4: _t->on_sequenceTableWidget_cellDoubleClicked((*reinterpret_cast< int(*)>(_a[1])),(*reinterpret_cast< int(*)>(_a[2]))); break;
        case 5: _t->on_evaluatePushButton_clicked(); break;
        case 6: _t->on_COBTestPushButton_clicked(); break;
        case 7: _t->on_startSchedulerPushButton_clicked(); break;
        case 8: _t->on_stopSchedulerPushButton_clicked(); break;
        case 9: _t->doSchedulerWork(); break;
        case 10: _t->updateTime(); break;
        case 11: _t->on_showNonSchedcheckBox_toggled((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 12: _t->on_testPlannerPushButton_clicked(); break;
        case 13: _t->on_enableDebugPushButton_clicked(); break;
        case 14: _t->on_GUIradioButton1_clicked(); break;
        case 15: _t->on_GUIradioButton2_clicked(); break;
        case 16: _t->on_GUIradioButton3_clicked(); break;
        case 17: _t->on_GUIradioButton4_clicked(); break;
        case 18: _t->on_speedSpinBox_valueChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
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
        if (_id < 19)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 19;
    }
    return _id;
}
QT_END_MOC_NAMESPACE
