#include <sstream>
#include <string>
#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Int8MultiArray.h>


#include "getgraph.h"

#define NUM_MAX_ROBOTS 32

typedef unsigned int uint;
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class PatrolAgent {

protected:
    
    int TEAMSIZE;
    int ID_ROBOT;

    double xPos[NUM_MAX_ROBOTS]; //tabelas de posições (atençao ao index pro caso de 1 so robot)
    double yPos[NUM_MAX_ROBOTS]; //tabelas de posições (atençao ao index pro caso de 1 so robot)

    tf::TransformListener *listener;

    std::string graph_file;
    uint dimension; // Graph Dimension
    uint current_vertex; // current vertex
    bool ResendGoal; // Send the same goal again (if goal failed...)
    bool interference;
    bool goal_complete;
    bool initialize;
    bool end_simulation;
    int next_vertex;
    uint backUpCounter;
    vertex *vertex_web;
    double *instantaneous_idleness;
    double *last_visit;

    MoveBaseClient *ac;
    
    //GBS: To calculate robot's state:
    bool arrived;
    uint vertex_arrived;
    int robot_arrived;

    //SEBS: To calculate robot's state:
    bool intention;
    uint vertex_intention;
    int robot_intention;

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
        arrived = false;
        ac = NULL;
    }
    
    virtual void init(int argc, char** argv);
    void initialize_node();
    void update_idleness();
    
    virtual void run();
    
    void getRobotPose(int robotid, float &x, float &y, float &theta);
    void odomCB(const nav_msgs::Odometry::ConstPtr& msg);
    
    void sendGoal(MoveBaseClient *ac, double target_x, double target_y);
    
    void goalDoneCallback(const actionlib::SimpleClientGoalState &state, const move_base_msgs::MoveBaseResultConstPtr &result);
    void goalActiveCallback();
    void goalFeedbackCallback(const move_base_msgs::MoveBaseFeedbackConstPtr &feedback);

    
    bool check_interference (int ID_ROBOT);
    void do_interference_behavior();
    void backup();
    
    // Robot-Robot Communication
    void send_positions();
    void receive_positions();
    void send_results();
    void receive_results(int *vres);
    void send_interference();
    void positionsCB(const nav_msgs::Odometry::ConstPtr& msg);
    void resultsCB(const std_msgs::Int8MultiArray::ConstPtr& msg);
    
    // Must be implemented by sub-classes
    virtual int compute_next_vertex() = 0;

};


