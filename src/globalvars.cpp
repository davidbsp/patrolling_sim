#include "globalvars.h"

int TEAMSIZE;
int ID_ROBOT;

double xPos[NUM_MAX_ROBOTS]; //tabelas de posições (atençao ao index pro caso de 1 so robot)
double yPos[NUM_MAX_ROBOTS]; //tabelas de posições (atençao ao index pro caso de 1 so robot)

tf::TransformListener *listener=NULL;

bool ResendGoal; //Send the same goal again (if goal failed...)
bool interference;
bool goal_complete;
bool initialize = true;
bool end_simulation = false;
int next_vertex = -1;
uint backUpCounter;
uint goalvertex;

//GBS: To calculate robot's state:
bool arrived = false;
uint vertex_arrived;
int robot_arrived;

//SEBS: To calculate robot's state:
bool intention = false;
uint vertex_intention;
int robot_intention;

ros::Subscriber odom_sub;
ros::Publisher odom_pub;
ros::Subscriber results_sub;
ros::Publisher results_pub;
ros::Publisher cmd_vel_pub;

