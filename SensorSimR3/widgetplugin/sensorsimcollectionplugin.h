#ifndef SensorSimCollectionPlugin_H
#define SensorSimCollectionPlugin_H


//#include <QtDesigner/QtDesigner>
//#include <QtCore/qplugin.h>
#include <QDesignerCustomWidgetCollectionInterface>

class SensorSimCollectionPlugin : public QObject, public QDesignerCustomWidgetCollectionInterface
{
    Q_OBJECT
    Q_INTERFACES(QDesignerCustomWidgetCollectionInterface)
public:
    explicit SensorSimCollectionPlugin(QObject *parent = 0);
    virtual QList<QDesignerCustomWidgetInterface*> customWidgets() const;

private:
     QList<QDesignerCustomWidgetInterface*> widgets;
    
signals:
    
public slots:
    
};

#endif // SensorSimCollectionPlugin_H
