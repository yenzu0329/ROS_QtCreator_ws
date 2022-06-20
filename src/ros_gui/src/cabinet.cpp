#include "cabinet.h"
QBrush Cabinet::brush(Qt::lightGray);
QPen   Cabinet::pen(Qt::black, 2);
int    Cabinet::count = 0;

Cabinet::Cabinet(qreal x, qreal y, qreal width, qreal height, QString direction)
{
    id = count++;
    buying_items = 0;
    rect.setSize(QSize(width, height));
    if(direction == "up")
    {
        pos = QPointF(x+width/2, y-35);
        dir = -M_PI_2;
    }
    else if(direction == "down")
    {
        pos = QPointF(x+width/2, y+height+35);
        dir = M_PI_2;
    }
    else if(direction == "right")
    {
        pos = QPointF(x+width+35, y+height/2);
        dir = M_PI;
    }
    else // direction == "left"
    {
        pos = QPointF(x-35, y+height/2);
        dir = 0;
    }
    setPos(x, y);
    setCursor(Qt::PointingHandCursor);
    setFlag(QGraphicsItem::ItemIsSelectable, true);
}

void Cabinet::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget)
{
    Q_UNUSED(option);
    Q_UNUSED(widget);

    painter->setPen(pen);
    painter->setBrush(brush);
    painter->drawRect(rect);
    painter->drawText(QPointF(2,rect.height()-3),QString::number(id));
    painter->setPen(Qt::red);
    painter->setBrush(Qt::red);
    for(int i=0; i<buying_items; i++)
    {
        int col = int(rect.width()-5)/10;
        int x = 5 + (i % col)*10;
        int y = 5 + (i / col)*10;
        painter->drawEllipse(x, y, 5, 5);
    }
}

QRectF Cabinet::boundingRect() const
{
    return QRectF(rect);
}

QPointF Cabinet::getPos(){return pos;}
qreal   Cabinet::getDir(){return dir;}
int     Cabinet::getID() {return id;}
void    Cabinet::setBuyingItem(int i)
{
    if(i==0)    buying_items = 0;
    else        buying_items += i;
}
int     Cabinet::getBuyingItem(){return buying_items;}

