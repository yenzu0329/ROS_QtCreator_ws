#ifndef CABINET_H
#define CABINET_H

#include <QGraphicsItem>
#include <QPainter>
#include <QPen>
#include <QBrush>
#include <QCursor>
#include <QtMath>
#include <QFile>

class Cabinet: public QGraphicsItem
{
public:
    Cabinet(qreal x, qreal y, qreal width, qreal height, QString direction);
    QPointF getPos();
    qreal getDir();
    int getID();
    void setBuyingItem(int i);
    int  getBuyingItem();

private:
    QRectF rect;
    QPointF pos;
    qreal dir;
    int id;
    int buying_items;
    static QBrush brush;
    static QPen   pen;
    static int    count;
private slots:
    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget)override;
    QRectF boundingRect() const override;
};

#endif // CABINET_H
