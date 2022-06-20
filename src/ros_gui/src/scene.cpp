#include "scene.h"

bool draw_path = false, init_pos = false;
double px, py, qx, qy, qz, qw, yaw;
Scene::Scene(qreal x, qreal y, qreal width, qreal height): QGraphicsScene(x, y, width, height)
{
    setItemIndexMethod(QGraphicsScene::NoIndex);

    // set background
    map = new QPixmap(":/map.png");
    map_item = new QGraphicsPixmapItem(*map);
    qreal map_width = map->width();
    qreal scale = width / map_width;
    map_item->setScale(scale);
    addItem(map_item);

    // setup cabinet
    QFile file(":/cabinet.txt");
    QTextStream in(&file);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text))  return;
    QString line = in.readLine();
    QStringList words;
    while(!(line = in.readLine()).isNull())
    {
        words = line.split(" ", QString::SkipEmptyParts);
        if(words.length() != 5) continue;
        qreal x = words[0].toDouble();
        qreal y = words[1].toDouble();
        qreal w = words[2].toDouble();
        qreal h = words[3].toDouble();
        QString dir = words[4];
        Cabinet *cab = new Cabinet(x, y, w, h, dir);
        cabinets.push_back(cab);
        addItem(cab);
    }
    file.close();

    // setup robot
    nh.reset(new ros::NodeHandle("~"));
    sub_model_states = nh->subscribe<gazebo_msgs::ModelStates>("/gazebo/model_states", 1, &Scene::feedBackPos, this);
    robot = new Robot;
    addItem(robot);

    // setup astar
    pub_dests = nh->advertise<ros_gui::Paths>("/ros_gui/dests", 1);
    sub_paths = nh->subscribe<ros_gui::Paths>("/ros_gui/paths", 5, &Scene::addPath, this);
}

Scene::~Scene()
{
    delete map_item;
    delete map;
}

void Scene::mousePressEvent(QGraphicsSceneMouseEvent *event)
{
    QGraphicsScene::mousePressEvent(event);
    if(draw_path)
    {
        QList<QGraphicsItem *> selects = selectedItems();
        if(!selects.empty())
        {
            if(dynamic_cast<Cabinet *>(selects.first()))
            {
                draw_path = false;
                path->compile((Cabinet *)selects.first());
                paths.push_back(path);
                qDebug() << "finish";
            }
        }
        else
            path->addEndPoint(event->scenePos());
    }
    else
    {
        if(robot->isSelected())
        {
            path = new Path(robot->pos());
            addItem(path);
            draw_path = true;
        }
    }
}

void Scene::mouseMoveEvent(QGraphicsSceneMouseEvent *event)
{
    QGraphicsScene::mouseMoveEvent(event);
    if(draw_path)
        path->changeLastPoint(event->scenePos());
    update();
}

void Scene::mouseReleaseEvent(QGraphicsSceneMouseEvent *event)
{
    QGraphicsScene::mouseReleaseEvent(event);
}

void Scene::compile()
{
    if(!run_Btn->isChecked())
        robot->send_msg();

}

void Scene::showGoods()
{
    for(int i=0; i<cabinets.length(); i++)
        cabinets[i]->setBuyingItem(0);

    for(int i=0; i<goods.length(); i++)
    {
        int id = goods[i]->id;
        cabinets[id]->setBuyingItem(goods[i]->num->value());
    }
}

void Scene::feedBackPos(const gazebo_msgs::ModelStates::ConstPtr& msg)
{
    if(run_Btn->isChecked())
    {
      // [1] = mars_lite
      unsigned long i = msg->pose.size()-1;
      px = msg->pose[i].position.x * 100;
      py = msg->pose[i].position.y * 100;
      qx = msg->pose[i].orientation.x;
      qy = msg->pose[i].orientation.y;
      qz = msg->pose[i].orientation.z;
      qw = msg->pose[i].orientation.w;
      yaw = quaternion2Yaw(qx, qy, qz, qw);
      if(!init_pos)
      {
        init_pos = true;
        robot->setInitPos(px, py, yaw);
      }
      robot->changePos(px, py, yaw);
    }
}

inline double Scene::quaternion2Yaw(double x, double y, double z, double w)
{
    return atan(2*(x*y+w*z)/(1-2*(y*y+z*z)));
}

void Scene::spinOnce()
{
    if(ros::ok())   ros::spinOnce();
    else            return;
}

void Scene::startAstar()
{
    paths.clear();
    msg.point.clear();

    // send start pos
    ros_gui::PointXY point;
    point.x = robot->pos().x();
    point.y = robot->pos().y();
    point.cab_id = 0;
    msg.point.push_back(point);

    // send dest list
    for(int i=0; i<cabinets.length(); i++)
    {
        if(cabinets[i]->getBuyingItem())
        {
            QPointF cab_pos = cabinets[i]->getPos();
            point.x = cab_pos.x();
            point.y = cab_pos.y();
            point.cab_id = i;
            msg.point.push_back(point);
        }
    }
    if(msg.point.size()==1)  run_Btn->click();
    pub_dests.publish(msg);
}

void Scene::addPath(const ros_gui::Paths::ConstPtr& msg)
{
    unsigned long len = msg->point.size();
    if(len == 1)
    {
        paths.push_back(nullptr);
        qDebug() << paths;
        return;
    }
    int idx = int(msg->point[0].cab_id);
    QVector<QPointF> endPoints;
    QPointF p;
    for(unsigned long i=0; i<len; i++)
    {
        p.setX(msg->point[i].x);
        p.setY(msg->point[i].y);
        endPoints.push_back(p);
    }
//    qDebug() << endPoints;
    path = new Path(endPoints);
    path->compile(cabinets[idx]);
    paths.push_back(path);
}
