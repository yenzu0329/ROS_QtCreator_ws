#include "robot.h"

bool rotate_or_move = true;
bool enter_point = true;
bool recover = false;

Robot::Robot()
{
    pic.load(":/robot.png");
    qreal scale = 38./pic.width();
    setScale(scale);
    setPos(QPointF(880, 600));
    setCursor(Qt::PointingHandCursor);
    setFlag(QGraphicsItem::ItemIsSelectable, true);
    setZValue(1);

    curr_ang = 0;
    path = nullptr;

    // setup msg
    pub = nh->advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    msg.linear.x = 0;
    msg.linear.y = 0;
    msg.linear.z = 0;
    msg.angular.x = 0;
    msg.angular.y = 0;
    msg.angular.z = 0;
}

void Robot::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget)
{
    Q_UNUSED(option);
    Q_UNUSED(widget);

    painter->rotate(-curr_ang * 180 / M_PI);
    painter->drawPixmap(-pic.width()/2, -pic.height()/2, pic);
    update();
}

QRectF Robot::boundingRect() const
{
    return QRectF(-pic.width()/2, -pic.height()/2, pic.width(), pic.height());
}

void Robot::setPath(Path *path) {this->path = path;}
void Robot::setInitPos(double px, double py, double yaw)
{
    init_px = px;
    init_py = py;
    curr_ang = yaw;
}
void Robot::changePos(double px, double py, double yaw)
{
    if(path == nullptr)
    {
      if(!paths.empty())
      {
          path = paths.first();
          if(path == nullptr)
          {
              run_Btn->click();
              while(!goods.empty())
                goods[0]->delete_btn->click();
              paths.clear();
          }
      }
      //else run_Btn->click();
      enter_point = true;
      send_msg(0, 0);
      return;
    }
    if(curr_ang > M_PI_2+0.05)
      curr_ang = (yaw > 0) ? (yaw - M_PI) : (yaw + M_PI);
    else if(curr_ang < -M_PI_2-0.05)
      curr_ang = (yaw > 0) ? (yaw - M_PI) : (yaw + M_PI);
    else if(curr_ang > M_PI_4 && yaw < -M_PI_4)
      curr_ang = yaw + M_PI;
    else if(curr_ang < -M_PI_4 && yaw > M_PI_4)
      curr_ang = yaw - M_PI;
    else
      curr_ang = yaw;
    qreal curr_x = 880 + (px - init_px)/2;
    qreal curr_y = 600 - (py - init_py)/2;
    setPos(curr_x, curr_y);

    qreal goal_ang = path->getAng(0);
    QPointF goal_pos = path->getPos(0);
    double dis = distance(goal_pos, pos());
    double diff_ang;

    if(goal_ang == -10 || goal_pos == QPointF(0, 0))
    {
      int dest_cab_id = path->arrive();
      path = nullptr;
      paths.pop_front();
      for(int i=0; i<goods.length(); i++)
      {
        if(goods[i]->id == dest_cab_id)
          goods[i]->done->setChecked(true);
      }
      return;
    }
    if(dis < 5 || enter_point)
    {
      diff_ang = diffAng(goal_ang);
      double rate = (diff_ang < ang_vel*0.25 && diff_ang > -ang_vel*0.25) ? 0.1 : 1;
      if(diff_ang > 0.01)       send_msg(0, ang_vel*rate);
      else if(diff_ang < -0.01) send_msg(0, -ang_vel*rate);
      else
      {
        if(enter_point)
        {
          path->popPos();
          enter_point = false;
        }
        else
        {
          path->popAng();
          enter_point = true;
        }
        send_msg(0, 0);
      }
    }
    else
    {
      double rate = (dis < vel*20) ? 0.1 : 1;
      goal_ang = atan2(-goal_pos.y()+curr_y, goal_pos.x()-curr_x);
      diff_ang = diffAng(goal_ang);
      if(diff_ang > 0.2 || diff_ang < -0.2 || recover)
      {
        if(diff_ang < 0.01 && diff_ang > -0.01)  recover = false;
        else                                     recover = true;
        if(diff_ang < 0)  send_msg(0, -0.5);
        else              send_msg(0, 0.5);
      }
      else
        send_msg(vel*rate, diff_ang*10);
    }
    //qDebug() << "goal pos: " << goal_pos << "pos: " << pos() << "goal ang: " << goal_ang << "ang: " << curr_ang;

}

void Robot::send_msg(double vel, double arg_vel)
{
    if(msg.linear.x != vel || msg.angular.z != arg_vel)
    {
      msg.linear.x = vel;
      msg.angular.z = arg_vel;
      pub.publish(msg);
    }
}

double Robot::diffAng(double goal_ang)
{
  double diff_ang;
  diff_ang = goal_ang - curr_ang;
  diff_ang = (diff_ang > 0) ? diff_ang : (diff_ang + M_PI*2);
  if(diff_ang > M_PI)
    diff_ang = diff_ang - M_PI*2;
  return diff_ang;
}
