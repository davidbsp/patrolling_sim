/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2014, ISR University of Coimbra.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the ISR University of Coimbra nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Author: Luca Iocchi (2014-2016)
*********************************************************************/

#include <sstream>
#include <string>
#include <vector>
#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Int16MultiArray.h>


#include "getgraph.h"

#define NUM_MAX_ROBOTS 32
#define INTERFERENCE_DISTANCE 2

#include "message_types.h"

typedef unsigned int uint;
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class PatrolAgent {

protected:
    
    int TEAMSIZE;
    int ID_ROBOT;

    double xPos[NUM_MAX_ROBOTS]; //tabelas de posições (atençao ao index pro caso de 1 so robot)
    double yPos[NUM_MAX_ROBOTS]; //tabelas de posições (atençao ao index pro caso de 1 so robot)

    tf::TransformListener *listener;

    std::string graph_file, mapname;
    uint dimension; // Graph Dimension
    uint current_vertex; // current vertex
    bool ResendGoal; // Send the same goal again (if goal failed...)
    bool interference;
    double last_interference;
    bool goal_complete;
    bool initialize;
    bool end_simulation;
    int next_vertex;
    // uint backUpCounter;
    vertex *vertex_web;
    double *instantaneous_idleness;  // local idleness
    double *last_visit;
    std::vector<int> vresults; // results exchanged among robots
    bool goal_canceled_by_user;
    double goal_reached_wait, communication_delay, last_communication_delay_time, lost_message_rate;
    std::string initial_positions;
    int aborted_count, resend_goal_count;
    
    MoveBaseClient *ac; // action client for reaching target goals
    
    ros::Subscriber odom_sub, positions_sub;
    ros::Publisher positions_pub;
    ros::Subscriber results_sub;
    ros::Publisher results_pub;
    ros::Publisher cmd_vel_pub;

    
public:
    
    PatrolAgent() { 
        listener=NULL;
        next_vertex = -1;
        initialize = true;
        end_simulation = false;
        ac = NULL;
    }
    
    virtual void init(int argc, char** argv);
    void ready();
    void initialize_node();
    void readParams(); // read ROS parameters
    void update_idleness();  // local idleness
    
    virtual void run();
    
    void getRobotPose(int robotid, float &x, float &y, float &theta);
    void odomCB(const nav_msgs::Odometry::ConstPtr& msg);
    
    void sendGoal(int next_vertex);
    void cancelGoal();
    
    void goalDoneCallback(const actionlib::SimpleClientGoalState &state, const move_base_msgs::MoveBaseResultConstPtr &result);
    void goalActiveCallback();
    void goalFeedbackCallback(const move_base_msgs::MoveBaseFeedbackConstPtr &feedback);

    
    void send_goal_reached();
    bool check_interference (int ID_ROBOT);
    void do_interference_behavior();
    void backup();
    
    void onGoalNotComplete(); // what to do when a goal has NOT been reached (aborted)
    
    // Events
    virtual void onGoalComplete(); // what to do when a goal has been reached
    virtual void processEvents();  // processes algorithm-specific events
    
    // Robot-Robot Communication
    void send_positions();
    void receive_positions();
    virtual void send_results();  // when goal is completed
    virtual void receive_results();  // asynchronous call
    void do_send_message(std_msgs::Int16MultiArray &msg);
    void send_interference();
    void positionsCB(const nav_msgs::Odometry::ConstPtr& msg);
    void resultsCB(const std_msgs::Int16MultiArray::ConstPtr& msg);
    
    // Must be implemented by sub-classes
    virtual int compute_next_vertex() = 0;

};


