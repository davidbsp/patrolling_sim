#include <sstream>
#include <string>
#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Int8MultiArray.h>

#include "PatrolAgent.h"

class Random_Agent: public PatrolAgent {
    
public:
    virtual int compute_next_vertex();
    
};


int Random_Agent::compute_next_vertex() {
    // Random algorithm
    
    //number of neighbors of current vertex (number of existing possibilites)
    uint num_neighs = vertex_web[current_vertex].num_neigh;
    uint next_vertex;
    
    srand ( time(NULL) );
    int i = rand() % num_neighs;
    next_vertex = vertex_web[current_vertex].id_neigh[i];
    
    ROS_INFO("Random choice: %d",next_vertex);
    
    return next_vertex;    
}

int main(int argc, char** argv) {
  
    Random_Agent agent;
    agent.init(argc,argv);
    agent.run();

    return 0; 
}
