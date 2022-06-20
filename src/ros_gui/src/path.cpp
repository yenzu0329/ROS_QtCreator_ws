#include "path.h"

double distance(QPointF p1, QPointF p2)
{
    QPointF p = p1-p2;
    return sqrt(pow(p.x(),2) + pow(p.y(),2));
}

Path::Path(QPointF start_pos)
{
    endPoints.push_back(start_pos);
    endPoints.push_back(start_pos);
}

Path::Path(QVector<QPointF> endPoints)
{
    this->endPoints = endPoints;
}

void Path::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget)
{
    Q_UNUSED(option);
    Q_UNUSED(widget);
    painter->setPen(QPen(Qt::red, 3));
    for(int i=0; i<endPoints.length()-1; i++)
    {
        painter->drawLine(endPoints[i], endPoints[i+1]);
    }
}

QRectF Path::boundingRect() const
{
    QPointF topLeftPos(5000, 5000);
    QPointF bottomRightPos(0, 0);
    for (QPointF p: qAsConst(endPoints))
    {
        //set topLeftPos
        if(p.x() < topLeftPos.x())
            topLeftPos.setX(p.x());
        if(p.y() < topLeftPos.y())
            topLeftPos.setY(p.y());

        //set bottomRightPos
        if(p.x() > bottomRightPos.x())
            bottomRightPos.setX(p.x());
        if(p.y() > bottomRightPos.y())
            bottomRightPos.setY(p.y());
    }
    return QRectF(topLeftPos-QPointF(3,3), bottomRightPos+QPointF(3,3));
}

void Path::changeLastPoint(QPointF pos)
{
    endPoints.last() = pos;
}

void Path::changeFirstPoint(QPointF pos)
{
    endPoints[0] = pos;
}

void Path::addEndPoint(QPointF pos)
{
    endPoints.push_back(pos);
}

void Path::compile(Cabinet *cab)
{
    dest_cab = cab;
    endPoints.last() = cab->getPos();
    if(endPoints.length() == 2 && distance(endPoints[0], endPoints[1])<10)
    {
        // same point
        endPoints.clear();
        angles.clear();
        return;
    }
    for(int i=0; i<endPoints.length()-1; i++)
    {
        qreal angle = qAtan2(-endPoints[i+1].y()+endPoints[i].y(),endPoints[i+1].x()-endPoints[i].x());
        angles.push_back(angle);
    }
    angles.push_back(cab->getDir());
    qDebug() << endPoints;
    qDebug() << angles;
}

QPointF Path::getPos(int idx)
{
    //idx++;
    if(idx < endPoints.length())    return endPoints[idx];
    else                            return QPointF(0, 0);
}

qreal Path::getAng(int idx)
{
    if(idx < angles.length())   return angles[idx];
    else                        return -10;
}

void Path::popAng(){angles.pop_front();}
void Path::popPos(){endPoints.pop_front();}
int  Path::arrive()
{
  dest_cab->setBuyingItem(0);
  return dest_cab->getID();
}
