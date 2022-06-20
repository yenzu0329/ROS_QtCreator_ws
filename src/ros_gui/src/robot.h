#ifndef ROBOT_H
#define ROBOT_H

#include <QGraphicsItem>
#include <QPixmap>
#include <QPainter>
#include <QPen>
#include <QCursor>
#include <QtMath>
#include "path.h"
#include "global_var.h"
#include "geometry_msgs/Twist.h"

class Robot: public QGraphicsItem
{
public:
    Robot();
    void setPath(Path* path);
    bool move(double &v, double &av);
    void compile();
    void changePos(double px, double py, double yaw);
    void setInitPos(double px, double py, double yaw);
    void send_msg(double vel = 0, double arg_vel = 0);
    double diffAng(double goal_ang);

private:
    double curr_ang;
    double init_px, init_py;
    QPixmap pic;
    Path *path;
    QVector<int> steps; //rotate->move->rotate->move;
    ros::Publisher pub;
    geometry_msgs::Twist msg;

private slots:
    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget)override;
    QRectF boundingRect() const override;
};
#endif // ROBOT_H
