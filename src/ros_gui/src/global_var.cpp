#include "global_var.h"

QPushButton* run_Btn;
QList<Good *> goods;
QList<Path *> paths;
ros::NodeHandlePtr nh;

const int freq = 150;
double vel;
double ang_vel;
int goods_count = 0;
