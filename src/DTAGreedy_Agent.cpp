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
#include <pthread.h>
#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Int16MultiArray.h>

#include "PatrolAgent.h"
#include "algorithms.h"
#include "config.h"

#define CONFIG_FILENAME "params/DTA/DTAGreedy.params"

class DTAGreedy_Agent: public PatrolAgent {
private:
    double *global_instantaneous_idleness;  // global estimated idleness
    double last_update_idl;
    ConfigFile cf;
    double theta_idl, theta_cost, theta_odist;
    float origin_x, origin_y, origin_theta;
    pthread_mutex_t lock;
    
public:
    DTAGreedy_Agent() : cf(CONFIG_FILENAME)
    {   // lock = PTHREAD_MUTEX_INITIALIZER; 
        pthread_mutex_init(&lock, NULL);
        
    }
    
    virtual void init(int argc, char** argv);
    virtual int compute_next_vertex();
    virtual void send_results();
    virtual void receive_results();    
    
    double compute_cost(int vertex);
    double distanceFromOrigin(int vertex);
    double utility(int vertex);
    void update_global_idleness();
};

void DTAGreedy_Agent::init(int argc, char** argv) {
    
    printf("DTAGreedy_Agent::init\n");
    
    PatrolAgent::init(argc,argv);
  
    global_instantaneous_idleness = new double[dimension];
    for(size_t i=0; i<dimension; i++) {
        global_instantaneous_idleness[i]=100;  // start with a high value    
    }
        
    last_update_idl = ros::Time::now().toSec();
    
    theta_idl = cf.getDParam("theta_idleness");
    theta_cost = cf.getDParam("theta_navigation");
    theta_odist = cf.getDParam("theta_distance_from_origin");
    
    std::stringstream paramss;
    paramss << theta_idl << "," << theta_cost << "," << theta_odist;

    ros::param::set("/algorithm_params",paramss.str());

    int value = ID_ROBOT;
    if (value==-1){value=0;}
    getRobotPose(value,origin_x, origin_y, origin_theta);
    ROS_INFO("Robot %d: Initial pose %.1f %.1f %.1f",value,origin_x, origin_y, origin_theta);
    
}

double DTAGreedy_Agent::compute_cost(int vertex)
{
    uint elem_s_path;
    int *shortest_path = new int[dimension]; 
    int id_neigh;
    
    dijkstra( current_vertex, vertex, shortest_path, elem_s_path, vertex_web, dimension); //structure with normal costs
    double distance = 0;
    
    for(uint j=0; j<elem_s_path; j++){
        if (j<elem_s_path-1){
            id_neigh = is_neigh(shortest_path[j], shortest_path[j+1], vertex_web, dimension);
            distance += vertex_web[shortest_path[j]].cost[id_neigh];
        }       
    }
    
    return distance;
}


double DTAGreedy_Agent::distanceFromOrigin(int vertex) {
    double x = vertex_web[vertex].x, y = vertex_web[vertex].y;           
    return sqrt((origin_x-x)*(origin_x-x)+(origin_y-y)*(origin_y-y));
}
        
double DTAGreedy_Agent::utility(int vertex) {
    
    double idl = global_instantaneous_idleness[vertex];
    double cost = compute_cost(vertex);
    double odist = distanceFromOrigin(vertex);
    double U = theta_idl * idl + theta_cost * cost + theta_odist * odist;
    printf("   -- U[%d] ( %.1f, %.1f, %.1f ) = ( %.1f, %.1f, %.1f ) = %.1f\n",vertex,idl,cost,odist,theta_idl*idl,theta_cost*cost,theta_odist*odist,U);
    return U;
}

void DTAGreedy_Agent::update_global_idleness() 
{   
    double now = ros::Time::now().toSec();
    
    pthread_mutex_lock(&lock);
    for(size_t i=0; i<dimension; i++) {
        global_instantaneous_idleness[i] += (now-last_update_idl);  // update value    
    }
    pthread_mutex_unlock(&lock);
    
    last_update_idl = now;
}

// current_vertex (goal just reached)
int DTAGreedy_Agent::compute_next_vertex() {
    
    update_global_idleness();
    global_instantaneous_idleness[current_vertex] = 0.0;
    
    // DTA Greedy    
    double maxUtility = -1e9;
    int i_maxUtility = 0;
        
    for(size_t i=0; i<dimension; i++){
        
        double U = utility(i);
        if (U > maxUtility && i!=current_vertex){
            maxUtility = U;
            i_maxUtility = i;
        }
        // printf("   -- U[%lu] = %.2f\n",i,U);
    }
    
    int nv = i_maxUtility; // vertex_web[current_vertex].id_neigh[i_maxUtility];

    printf("DTAGreedy: next vertex = %d (U = %.2f)\n",nv,maxUtility);
    
    return nv;
}



// current_vertex (goal just reached)
// next_vertex (next goal)
void DTAGreedy_Agent::send_results() {
    int value = ID_ROBOT;
    if (value==-1){value=0;}
    
    //result= [ID,msg_type,global_idleness[1..dimension],next_vertex]
    int msg_type = DTAGREEDY_MSG_TYPE;
    std_msgs::Int16MultiArray msg;
    msg.data.clear();
    msg.data.push_back(value);
    msg.data.push_back(msg_type);
    //printf("  ** sending [%d, %d, ",ID_ROBOT,msg_type);
    
    pthread_mutex_lock(&lock);
    
    for(size_t i=0; i<dimension; i++) {
        // convert in 1/10 of secs (integer value) Max value 3276.8 second (> 50 minutes) !!!
        int ms = (int)(global_instantaneous_idleness[i]*10);
        if (ms>32768) { // Int16 is used to send messages
            ROS_WARN("Wrong conversion when sending idleness value in messages!!!");
            ms=32000;
        }
        if ((int)i==next_vertex) ms=0;
        // printf("  ** sending GII[%lu] = %d from value %.2f \n",i,ms,global_instantaneous_idleness[i]);
        //printf("%d, ",ms);
        msg.data.push_back(ms);
    }
    msg.data.push_back(next_vertex);
    //printf(",%d]\n",next_vertex);
    pthread_mutex_unlock(&lock);
    
    do_send_message(msg);   
      
}


void DTAGreedy_Agent::receive_results() {
    //result= [ID,msg_type,global_idleness[1..dimension],next_vertex]
    
    double now = ros::Time::now().toSec();
    
    //printf("  ** here ** \n");
    
    std::vector<int>::const_iterator it = vresults.begin();
    int id_sender = *it; it++;
    int msg_type = *it; it++;
    
    //printf("  ** received [%d, %d, ... \n",id_sender,msg_type);
    int value = ID_ROBOT;
    if (value==-1){value=0;}
    
    if ((id_sender==value) || (msg_type!=DTAGREEDY_MSG_TYPE)) 
        return;
    pthread_mutex_lock(&lock);
    for(size_t i=0; i<dimension; i++) {
        int ms = *it; it++; // received value
        //printf("    -- received from %d remote-GII[%lu] = %d\n",id_sender,i,ms);
        //printf("  - %d - \n",ms);
        double rgi = (double)ms/10.0; // convert back in seconds
        //printf("  - i=%lu - \n",i);
        //printf("  - global...[i]=%.1f - \n",global_instantaneous_idleness[i]);
        if (isnan(global_instantaneous_idleness[i])) {
            printf("NAN Exiting!!!"); return;
        }

        global_instantaneous_idleness[i] = std::min(
            global_instantaneous_idleness[i]+(now-last_update_idl), rgi);
        //printf("    ++ GII[%lu] = %.1f (r=%.1f)\n",i,global_instantaneous_idleness[i],rgi);
    }
    pthread_mutex_unlock(&lock);
    last_update_idl = now;

    int sender_next_vertex = *it; it++;
    //printf(" ... %d]\n",sender_next_vertex);
    
    // interrupt path if moving to the same target node
    if (sender_next_vertex == next_vertex) { // two robots are going to the same node
        ROS_INFO("Robots %d and %d are both going to vertex %d",value,id_sender,next_vertex);
        ROS_INFO("Robot %d: STOP and choose another target",value);
        // change my destination
        cancelGoal(); // stop the current behavior
        current_vertex = next_vertex; // simulate that the goal vertex has been reached (not sent to the monitor)
        next_vertex = compute_next_vertex(); // compute next vertex (will be different from current vertex)
        sendGoal(next_vertex);
    }
    //printf("*** END ***\n");
}

int main(int argc, char** argv) {
  
    DTAGreedy_Agent agent;
    agent.init(argc,argv);
    agent.run();

    return 0; 
}
