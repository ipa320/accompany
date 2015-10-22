#include "locn.h"

locn::locn(QString txt,qreal x, qreal y, qreal sceneWidth, qreal sceneHeight, qreal orientation,bool editEnabled, bool oEnabled)
{
    setFlag(QGraphicsTextItem::ItemIsMovable);
    setFlag(QGraphicsItem::ItemIsSelectable);
    setFlag(QGraphicsItem::ItemIsFocusable);

   // setScale(1);


    sHeight = sceneHeight;
    sWidth = sceneWidth;

    _x = convertX(x);
    _y = convertY(y);

    setPos(_x,_y);
    setZValue(1);

    displayOrientation = convertOrientation(orientation);
    originalOrientation = orientation;

    if (oEnabled)
    {
        setRotation(displayOrientation);
    }
    else
    {
        setRotation(0);
    }

    setPlainText(makeText("<- " + txt,true,originalOrientation));

    setEditEnabled(editEnabled);
    setOrientationEnabled(oEnabled);

   //         qDebug()<<rotation();

}

void locn::setEditEnabled(bool editEnabled)
{
    if (editEnabled)
    {
      setTextInteractionFlags(Qt::TextEditable);
    }
    else
    {
      setTextInteractionFlags(Qt::NoTextInteraction);
    }
 }

void locn::setOrientationEnabled(bool oEnabled)
{
    orientationEnabled = oEnabled;

 }

qreal locn::convertX(qreal x)
{
   return (x+2)*sWidth/8;   // converts x to  robot coords

}

qreal locn::deConvertX(qreal x)
{
   return (8*x/sWidth)-2;  // converts robotcoords to  x

}

qreal locn::convertY(qreal y)
{
    return sHeight - (y+3)*sHeight/6; // converts y to  robot coords
}

qreal locn::deConvertY(qreal y)
{
    return ((y - sHeight)/(sHeight/6) + 3) * -1; // converts robotcoords to  y

}

qreal locn::getX()
{
    return _x;

}

qreal locn::getY()
{
    return _y;
}


QString locn::makeText(QString txt,bool convert,qreal orientation)
{

    QString xS,yS,orient;

    if (convert)
    {
        xS.setNum(deConvertX(pos().toPoint().x()));
        yS.setNum(deConvertY(pos().toPoint().y()));
    }
    else
    {
        xS.setNum(pos().toPoint().x());
        yS.setNum(pos().toPoint().y());
    }

    orient.setNum(orientation);

    return txt + " ("+xS+","+yS+"," + orient + ")";

}


void locn::mousePressEvent(QGraphicsSceneMouseEvent *event)
{
  //  qDebug()<<"Pressed";

    if (event->buttons() == Qt::RightButton)
    {
  //      rotate(-10);
    }



    setDefaultTextColor(Qt::red);

    QGraphicsTextItem::mousePressEvent(event);
}

void locn::wheelEvent(QGraphicsSceneWheelEvent *event)
{

    qDebug()<<"wheel"<<event->delta();

    qDebug()<<displayOrientation<<" "<<originalOrientation;

    if (orientationEnabled)
    {
        if (event->delta() > 0)
        {
            rotate(+1);
            displayOrientation+=1;
            if (originalOrientation > 0 || originalOrientation < 0)
            {
                originalOrientation-=1;
            }
            else  // it is zero
            {
                originalOrientation = 179;
            }

        }
        else
        {
            rotate(-1);
            displayOrientation-=1;
            if (originalOrientation > 0 || originalOrientation < 0)
            {
                originalOrientation+=1;

            }
            else
            {
               originalOrientation = -179;
            }
        }

        if (originalOrientation > 180 )
        {
            originalOrientation -= 360;
        }
        else
        {
            if (originalOrientation < -180 )
            {
                originalOrientation += 360;
            }
        }


        setPlainText(makeText(toPlainText().section(" (",0,0),true,originalOrientation));


        qDebug()<<"  " << displayOrientation<<" "<<originalOrientation;
   }

   QGraphicsTextItem::wheelEvent(event);

}

void locn::mouseReleaseEvent(QGraphicsSceneMouseEvent *event)
{
//    qDebug()<<"Released";

    setDefaultTextColor(Qt::black);

 //   qDebug()<<pos().x();
 //   qDebug()<<pos().y();

    if (pos().x() > 800) setPos(800,pos().y());
    if (pos().x() < 0) setPos(0,pos().y());

    if (pos().y() > 600) setPos(pos().x(),600);
    if (pos().y() < 0) setPos(pos().x(),0);

    setPlainText(makeText(toPlainText().section(" (",0,0),true,originalOrientation));

    QGraphicsTextItem::mouseReleaseEvent(event);
}

void locn::mouseMoveEvent(QGraphicsSceneMouseEvent *event)
{
//    qDebug()<<"Moving";

//    qDebug()<<toPlainText().section("(",0,0);

    setPlainText(makeText(toPlainText().section(" (",0,0),true,originalOrientation));

    QGraphicsTextItem::mouseMoveEvent(event);
}

void locn::mouseDoubleClickEvent(QGraphicsSceneMouseEvent *event)
{
//    qDebug()<<"Double";
}

qreal locn::convertOrientation(qreal orientation)
{
    // assumes rotation angle from robot between -180 to 0 to + 180

    if (orientation > 0 && orientation < 91)            // 1 to 90
    {
        return 180 - orientation;
    }
    else
    if (orientation < 0 && orientation > -91)          // -1 to -90
    {
        return -180 - orientation;
    }
    else
    if (orientation > 90 && orientation < 181)         // 91  to 180
    {
        return 180 - orientation;
    }
    else
    if (orientation < -90 && orientation > -181)      // -91  to -180
    {
        return -180 - orientation;
    }

    // 0 remains unchanged
}


