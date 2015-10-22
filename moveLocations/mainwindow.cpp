#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "VoronoiDiagramGenerator.h"

locn* locations[100];
int locIndex;


MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    closeDownRequest = false;

    sceneWidth = 800;
    sceneHeight = 600;

    openDB();

    if (closeDownRequest)
        return;

    scene = new QGraphicsScene(this);

    scene->setSceneRect(0,0,sceneWidth,sceneHeight);

    ui->graphicsView->setScene(scene);

    statusBar();

    printer = 0;

    orientationFlag = false;

    ui->l0checkBox->setChecked(true);
    ui->l1checkBox->setChecked(true);
    ui->l2checkBox->setChecked(true);
    ui->l3checkBox->setChecked(true);

    init();

}

void MainWindow::init()
{

    scene->clear();
    numVLines=0;

    ui->checkBox->setChecked(false);


 //   ui->checkBox_2->setChecked(false);

    // draw a rectangle to represent the room

    QAbstractGraphicsShapeItem *i = scene->addRect( QRectF(0,0,scene->width(),scene->height()) );
    i->setBrush( QColor("white") );
    i->setPen( QPen(QColor("blue"), 4) );
    i->setZValue(0);


    // highlight point 0,0

 //   QGraphicsEllipseItem* j = scene->addEllipse(200,300,20,20);

 //   j->setBrush( QColor("green") );
 //   j->setPen( QPen(QColor("red"), 4) );
 //   j->setZValue(3);

    // label X and Y locations

    labelX(-2);    labelX(-1);     labelX(0);
    labelX(1); labelX(2);     labelX(3);     labelX(4); labelX(5);     labelX(6);

    labelY(-2);    labelY(-1);     labelY(0);
    labelY(+1);     labelY(+2);

    // location name

    QGraphicsTextItem* l = new QGraphicsTextItem(locationName);
    l->setPos(0.0,-50.0);
    l->setDefaultTextColor("red");
    l->setScale(2);
    l->setFont(QFont());
    scene->addItem(l);

    // add walls etc.

    QSqlQuery query;

    //default in mm

    qreal maxX = 8000;
    qreal maxY = 6000;

    QString q = "SELECT max(x1),max(y1),max(x2),max(y2) from LocationMeasurements where id = " + experimentLocation; // + " and room = 'Living Room'";
    query.prepare(q);
    query.exec();
    while (query.next())
    {
        maxX = query.value(2).toDouble();
        if (query.value(0).toDouble() > maxX)
        {
            maxX = query.value(0).toDouble();
        }

        maxY = query.value(3).toDouble();
        if (query.value(1).toDouble() > maxY)
        {
            maxY = query.value(1).toDouble();
        }

    }

    q = "SELECT x1,y1,x2,y2 from LocationMeasurements where id = " + experimentLocation; // + " and room = 'Living Room'";
    query.prepare(q);
    query.exec();

    while (query.next())
    {

        qreal x1 = query.value(0).toDouble();
        qreal y1 = query.value(1).toDouble();
        qreal x2 = query.value(2).toDouble();
        qreal y2 = query.value(3).toDouble();

        qDebug()<<x1<<y1<<x2<<y2;


        x1 = convertWall(x1, maxX, scene->width());
        y1 = convertWall(y1, maxY, scene->height());

        x2 = convertWall(x2, maxX, scene->width());
        y2 = convertWall(y2, maxY, scene->height());




         scene->addLine(x1,y1,x2,y2,QPen(QColor("red"), 5));
    }

    // add locations

    q = "SELECT  L1.xCoord,L1.yCoord, L1.orientation,L1.locationId, L1.where, L2.where, L1.name, IF(STRCMP(L1.name,L2.name),L2.name,''), IF(STRCMP(L2.name,L3.name),L3.name,''), IF(STRCMP(L3.name,L4.name),L4.name,'')\
               FROM Locations L1,\
                    Locations L2,\
                    Locations L3,\
                    Locations L4\
               WHERE L2.locationId = L1.where\
                 AND L3.locationId = L2.where\
                 AND L4.locationId = L3.where\
                 AND L1.validRobotLocation = 1\
                 AND L1.locationId between " + startLocation + " and " + endLocation;

    qDebug()<< q;




 //   q = "SELECT name, xCoord,yCoord, locationId, orientation from Locations where locationId between " + startLocation + " and " + endLocation + " and ValidRobotLocation = 1";

    query.prepare(q);
    query.exec();

    locIndex = 0;

    while (query.next())
    {
   //     qDebug()<< query.value(0).toString()<< " " << query.value(1).toString() << " " << query.value(2).toString();

        QString locTxt = query.value(6).toString() + "[" + query.value(3).toString() + "]";
        qreal x = query.value(0).toDouble();
        qreal y = query.value(1).toDouble();
        qreal orientation = query.value(2).toDouble();

        if (ui->l0checkBox->isChecked())
        {
            if ( query.value(7).toString() =="" && query.value(8).toString() =="" && query.value(9).toString() =="")
            {
                addLocnToScene(locTxt, x, y, orientation);
            }
        }

        if (ui->l1checkBox->isChecked())
        {
            if ( query.value(7).toString() !="" && query.value(8).toString() =="" && query.value(9).toString() =="")
            {
                addLocnToScene(locTxt, x, y, orientation);
            }
        }

        if (ui->l2checkBox->isChecked())
        {
            if ( query.value(7).toString() !="" && query.value(8).toString() !="" && query.value(9).toString() =="")
            {
                addLocnToScene(locTxt, x, y, orientation);
            }
        }

        if (ui->l3checkBox->isChecked())
        {
            if ( query.value(7).toString() !="" && query.value(8).toString() !="" && query.value(9).toString() !="")
            {
                addLocnToScene(locTxt, x, y, orientation);
            }
        }


    }

}

void MainWindow::addLocnToScene(QString locnTxt, qreal x, qreal y, qreal orientation)
{
    txt = new locn(locnTxt,x,y,sceneWidth,sceneHeight,orientation,false,orientationFlag);
    scene->addItem(txt);

    locations[locIndex]=txt;
    locIndex++;
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::labelX(qreal x)
{

    QString xx;
    QString text =  xx.number(x);

    x = (x+2)*scene->width()/8;

    QGraphicsTextItem* n = scene->addText(text);
    n->setPos(x,-20);
    n->setZValue(0);
}

void MainWindow::labelY(qreal y)
{

    QString yy;
    QString text =  yy.number(y);

    y = scene->height() - (y+3)*scene->height()/6;

    QGraphicsTextItem* n = scene->addText(text);
    n->setPos(-20,y);
    n->setZValue(0);
}



void MainWindow::openDB()
{
    bool ok;


    QFile file("../UHCore/Core/config.py");

    if (!file.exists())
    {
       qDebug()<<"No config.py found!!";
    }

    if (!file.open(QIODevice::ReadOnly | QIODevice::Text))
    {
        closeDownRequest = true;
        return;
    }

    QTextStream in(&file);
    while (!in.atEnd())
    {
       QString line = in.readLine();

       if (line.contains("mysql_log_user"))
       {
          user = line.section("'",3,3);
       }
       if (line.contains("mysql_log_password"))
       {
           pw = line.section("'",3,3);
       }
       if (line.contains("mysql_log_server"))
       {
          host = line.section("'",3,3);
       }
       if (line.contains("mysql_log_db"))
       {
          dBase = line.section("'",3,3);
       }
    }

    user = QInputDialog::getText ( this, "Accompany DB", "User:",QLineEdit::Normal,
                                     user, &ok);
    if (!ok)
    {
       closeDownRequest = true;
       return;
    }

    pw = QInputDialog::getText ( this, "Accompany DB", "Password:", QLineEdit::Password,
                                                                      pw, &ok);
    if (!ok)
    {
       closeDownRequest = true;
       return;
    }


    host = QInputDialog::getText ( this, "Accompany DB", "Host:",QLineEdit::Normal,
                                     host, &ok);
    if (!ok)
    {
      closeDownRequest = true;
      return;
    };

    dBase = QInputDialog::getText ( this, "Accompany DB", "Database:",QLineEdit::Normal,
                                     dBase, &ok);
    if (!ok)
    {
      closeDownRequest = true;
      return;
    };


    db = QSqlDatabase::addDatabase("QMYSQL");

    db.setHostName(host);
    db.setDatabaseName(dBase);
    db.setUserName(user);
    db.setPassword(pw);

    dbOpen = db.open();

    if (!dbOpen)
    {

        QMessageBox msgBox;
        msgBox.setIcon(QMessageBox::Critical);

        msgBox.setText("Database error - login problem - see console log!");
        msgBox.exec();

        qCritical("Cannot open database: %s (%s)",
                  db.lastError().text().toLatin1().data(),
                  qt_error_string().toLocal8Bit().data());

        closeDownRequest = true;

        return;
    }
    else
    {
        qDebug() << "Database Opened";
    }

    // get experimental location


    QSqlQuery query("SELECT ExperimentalLocationId  FROM SessionControl WHERE SessionId = 1 LIMIT 1");

    if (query.next())
    {
       experimentLocation = query.value(0).toString();
    }
    else
    {
        QMessageBox msgBox;
        msgBox.setIcon(QMessageBox::Critical);

        msgBox.setText("Can find session control table!");
        msgBox.exec();
        closeDownRequest = true;
        return;
    }

    QString q = "SELECT startLocationRange,endLocationRange,location FROM ExperimentalLocation where id = " + experimentLocation;
    query.prepare(q);
    query.exec();

    if (query.next())
    {
        startLocation = query.value(0).toString();
        endLocation   = query.value(1).toString();
        locationName  = query.value(2).toString();
    }
    else
    {
        QMessageBox msgBox;
        msgBox.setIcon(QMessageBox::Critical);

        msgBox.setText("Can find experimental location control table!");
        msgBox.exec();
        closeDownRequest = true;
        return;
    }




}

void MainWindow::on_refreshButton_clicked()
{
       init();
}

void MainWindow::on_printButton_clicked()
{
   if ( !printer ) printer = new QPrinter;

   QPrintDialog dialog(printer, this);

   if(dialog.exec())
   {
       QPainter pp(printer);
       scene->render(&pp);
   }

}

void MainWindow::on_updateButton_clicked()
{
    locn* item;
    QSqlQuery query;

    QString key,xS,yS, orient;
    qreal x,y,o;

    for (int i=0;i<locIndex;i++)
    {
        item = locations[i];
        x = item->deConvertX(item->x());
        y = item->deConvertY(item->y());
        key = item->toPlainText().section("[",1,1).section("]",0,0);
        xS.setNum(x);
        yS.setNum(y);
        o=item->getOrientation();
        orient.setNum(o);

        QString newName = item->toPlainText().section("<- ",1,1).section("[",0,0);

        QString q = "UPDATE Locations SET name = '" + newName + "', xCoord = " + xS + ", yCoord = " + yS + ", orientation = " + orient + " WHERE locationId = " + key;

        qDebug()<<q;

        query.prepare(q);
        query.exec();

    }
}

void MainWindow::on_checkBox_clicked(bool checked)
{
    locn* item;


    for (int i=0;i<locIndex;i++)
    {
        item = locations[i];
        item->setEditEnabled(checked);
    }



}

void MainWindow::on_checkBox_2_clicked(bool checked)
{
    locn* item;

    orientationFlag = checked;

    for (int i=0;i<locIndex;i++)
    {
        item = locations[i];
        item->setOrientationEnabled(checked);
     }

        init();
}

qreal MainWindow::convertWall(qreal x, qreal maxWall, qreal sceneSize)
{
    return x /maxWall * sceneSize;
}

void MainWindow::on_newLocnPushButton_clicked()
{

    newLocation newLocn(db,startLocation,endLocation,this); //create a object directly

    if (newLocn.exec() == QDialog::Accepted)  //modal dialog
    {
        if (newLocn.getName() == "")
        {
            return;
        }

        QString id,r,p;

        id.setNum(newLocn.getId());
        r.setNum(newLocn.getValidRobot());
        p.setNum(newLocn.getValidPerson());

        QString q;
        q = "INSERT INTO Locations VALUES (" + id + ",'" + newLocn.getName() +
                "'," + newLocn.getParent().section("::",0,0) + ",0,0," + r  +
                "," + p + ",0," + newLocn.getNearest().section("::",0,0) + ")";

        qDebug()<<q;

        QSqlQuery query;
        query.prepare(q);
        query.exec();

        init();
    }


}


void MainWindow::on_zoomoutPushButton_clicked()
{
    ui->graphicsView->scale(0.9,0.9);
}

void MainWindow::on_zoomInPushButton_clicked()
{
    ui->graphicsView->scale(1.1,1.1);
}





void MainWindow::on_voronoiPushButton_clicked()
{

    for (int i=0;i<numVLines;i++)
    {
        scene->removeItem(vLines[i]);
    }

    numVLines = 0;

    locn* item;

    float xValues[locIndex];
    float yValues[locIndex];

    for (int i=0;i<locIndex;i++)
    {
        item = locations[i];
        xValues[i]=(float)item->x();//getX();
        yValues[i]=(float)item->y();//getY();

    }

    long count = locIndex;

    VoronoiDiagramGenerator vdg;
    vdg.generateVoronoi(xValues,yValues,count, 0,800,0,600,3);

    vdg.resetIterator();

    float x1,y1,x2,y2;


    while(vdg.getNext(x1,y1,x2,y2))
    {
        qDebug()<<x1<<y1<<x2<<y2;
        vLines[numVLines]=scene->addLine(x1,y1,x2,y2,QPen(QColor("green"), 5));
        numVLines++;
    }
}

void MainWindow::on_l0checkBox_clicked(bool checked)
{

}
