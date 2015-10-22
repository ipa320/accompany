#ifndef LOCN_H
#define LOCN_H

#include <QPainter>
#include <QGraphicsItem>
#include <QGraphicsSceneMoveEvent>
#include <QKeyEvent>
#include <QDebug>

class locn : public QGraphicsTextItem
{
public:
    locn(QString txt,qreal x, qreal y, qreal sceneWidth, qreal sceneHeight, qreal orientation=0,bool editEnabled=false,bool oEnabled = false);
    qreal deConvertX(qreal x);
    qreal deConvertY(qreal y);
    void setEditEnabled(bool editEnabled);
    void setOrientationEnabled(bool oEnabled);
    qreal getOrientation(){return originalOrientation;};
    qreal getX();
    qreal getY();

protected:

    void mousePressEvent(QGraphicsSceneMouseEvent *event);
    void mouseReleaseEvent(QGraphicsSceneMouseEvent *event);
    void mouseMoveEvent(QGraphicsSceneMouseEvent *event);
    void mouseDoubleClickEvent(QGraphicsSceneMouseEvent *event);
    void wheelEvent(QGraphicsSceneWheelEvent *event);

private:

    qreal sHeight;
    qreal sWidth;

    qreal _x;
    qreal _y;

    qreal convertX(qreal x);
    qreal convertY(qreal y);

    bool orientationEnabled;

    qreal originalOrientation;

    qreal displayOrientation;


    QString makeText(QString txt, bool convert, qreal orientation);

    qreal convertOrientation(qreal orientation);


};

#endif // LOCN_H
