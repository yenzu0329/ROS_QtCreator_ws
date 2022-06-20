#ifndef PATH_H
#define PATH_H

#include <QGraphicsItem>
#include <QPainter>
#include <QtMath>
#include <QDebug>
#include "cabinet.h"

class Path : public QGraphicsItem
{
public:
    Path(QPointF start_pos);
    Path(QVector<QPointF> endPoints);
    void changeLastPoint(QPointF pos);
    void changeFirstPoint(QPointF pos);
    void addEndPoint(QPointF pos);
    void popAng();
    void popPos();
    void compile(Cabinet *cab);
    int  arrive();
    QPointF getPos(int idx);
    qreal getAng(int idx);

private:
    QVector<QPointF> endPoints;
    QVector<qreal> angles;
    Cabinet *dest_cab;

private slots:
    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget)override;
    QRectF boundingRect() const override;
};

#endif // PATH_H
