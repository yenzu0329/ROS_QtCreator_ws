#ifndef SCENE_H
#define SCENE_H

#include <QGraphicsScene>
#include <QGraphicsItem>
#include <QGraphicsSceneMouseEvent>
#include <QPixmap>
#include <QFile>
#include "gazebo_msgs/ModelStates.h"
#include "ros_gui/Paths.h"
#include "global_var.h"
#include "robot.h"
#include "cabinet.h"
#include "path.h"

class Scene : public QGraphicsScene
{
public:
    Scene(qreal x, qreal y, qreal width, qreal height);
    ~Scene();
    void showGoods();
    void feedBackPos(const gazebo_msgs::ModelStates::ConstPtr& msg);
    void addPath(const ros_gui::Paths::ConstPtr& msg);
    void startAstar();
    double quaternion2Yaw(double x, double y, double z, double w);

private:
    QPixmap *map;
    QGraphicsPixmapItem *map_item;
    Robot *robot;
    Path *path;
    QList<Cabinet*> cabinets;
    ros::Subscriber sub_model_states;  // receive gazebo_msgs/ModelStates from /gazebo/model_states
    ros::Subscriber sub_paths;         // receive ros_gui/Paths.h to /ros_gui/paths
    ros::Publisher pub_dests;          // send ros_gui/Paths.h to /ros_gui/dests
    ros_gui::Paths msg;

public slots:
    void move_rob();
    void compile();
    void spinOnce();

private slots:
    void mousePressEvent(QGraphicsSceneMouseEvent *event) override;
    void mouseMoveEvent(QGraphicsSceneMouseEvent *event) override;
    void mouseReleaseEvent(QGraphicsSceneMouseEvent *event) override;

};

#endif // SCENE_H
