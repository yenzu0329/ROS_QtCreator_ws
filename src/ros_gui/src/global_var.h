#ifndef GLOBAL_VAR_H
#define GLOBAL_VAR_H

#include <QPushButton>
#include <QtMath>
#include <QDebug>
#include <ros/ros.h>
#include "good.h"
#include "path.h"

extern QPushButton *run_Btn;
extern QList<Good *> goods;
extern QList<Path *> paths;
extern const int freq;
extern double vel;
extern double ang_vel;
extern int goods_count;
extern double distance(QPointF p1, QPointF p2);
extern ros::NodeHandlePtr nh;

#endif // GLOBAL_VAR_H
