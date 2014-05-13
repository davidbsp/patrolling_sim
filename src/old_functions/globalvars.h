#ifndef _GLOBALVARS_
#define _GLOBALVARS_

#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>


#define NUM_MAX_ROBOTS 32

typedef unsigned int uint;
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

extern int TEAMSIZE;
extern int ID_ROBOT;

extern double xPos[NUM_MAX_ROBOTS]; //tabelas de posições (atençao ao index pro caso de 1 so robot)
extern double yPos[NUM_MAX_ROBOTS]; //tabelas de posições (atençao ao index pro caso de 1 so robot)

extern tf::TransformListener *listener;

extern bool ResendGoal; //Send the same goal again (if goal failed...)
extern bool interference;
extern bool goal_complete;
extern bool initialize;
extern bool end_simulation;
extern int next_vertex;
extern uint backUpCounter;
extern uint goalvertex;

extern bool arrived;
extern uint vertex_arrived;
extern int robot_arrived;

extern bool intention;
extern uint vertex_intention;
extern int robot_intention;

extern ros::Subscriber odom_sub;
extern ros::Publisher odom_pub;
extern ros::Subscriber results_sub;
extern ros::Publisher results_pub;
extern ros::Publisher cmd_vel_pub;



#endif
