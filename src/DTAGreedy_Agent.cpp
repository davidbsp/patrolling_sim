#include <sstream>
#include <string>
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
    double theta_idl, theta_cost;
    
public:
    DTAGreedy_Agent() : cf(CONFIG_FILENAME)
    {}
    
    virtual void init(int argc, char** argv);
    virtual int compute_next_vertex();
    virtual void send_results();
    virtual void receive_results();    
    
    double compute_cost(int vertex);
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
        
double DTAGreedy_Agent::utility(int vertex) {
    
    double idl = global_instantaneous_idleness[vertex];
    double cost = compute_cost(vertex);
    double U = theta_idl * idl + theta_cost * cost;
    printf("   -- U[%d] ( %.1f, %.1f ) = %.1f\n",vertex,idl,cost,U);
    return U;
}

void DTAGreedy_Agent::update_global_idleness() 
{   
    double now = ros::Time::now().toSec();
    
    for(size_t i=0; i<dimension; i++) {
        global_instantaneous_idleness[i] += (now-last_update_idl);  // update value    
    }
    
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
    //result= [ID,msg_type,global_idleness[1..dimension],next_vertex]
    int msg_type = DTAGREEDY_MSG_TYPE;
    std_msgs::Int16MultiArray msg;
    msg.data.clear();
    msg.data.push_back(ID_ROBOT);
    msg.data.push_back(msg_type);
    //printf("  ** sending [%d, %d, ",ID_ROBOT,msg_type);
    for(size_t i=0; i<dimension; i++) {
        // convert in 1/100 of secs (integer value)
        int ms = (int)(global_instantaneous_idleness[i]*100);
        if ((int)i==next_vertex) ms=0;
        //printf("  ** sending GII[%lu] = %d\n",i,ms);
        //printf("%d, ",ms);
        msg.data.push_back(ms);
    }
    msg.data.push_back(next_vertex);
    //printf(",%d]\n",next_vertex);
    
    results_pub.publish(msg);   
    ros::spinOnce();    
}


void DTAGreedy_Agent::receive_results() {
    //result= [ID,msg_type,global_idleness[1..dimension],next_vertex]
    
    double now = ros::Time::now().toSec();
    
    std::vector<int>::const_iterator it = vresults.begin();
    int id_sender = *it; it++;
    if (id_sender==ID_ROBOT) return;
    int msg_type = *it; it++;
    if (msg_type!=DTAGREEDY_MSG_TYPE) return;
    //printf("  ** received [%d, %d, ",id_sender,msg_type);
    for(size_t i=0; i<dimension; i++) {
        int ms = *it; it++; // received value
        // printf("  ** received from %d remote-GII[%lu] = %.1f\n",id_sender,i,ms);
        //printf("%d, ",ms);
        double rgi = (double)ms/100.0; // convert back in seconds
        global_instantaneous_idleness[i] = std::min(
            global_instantaneous_idleness[i]+(now-last_update_idl), rgi);
        // printf("   ++ GII[%lu] = %.1f (r=%.1f)\n",i,global_instantaneous_idleness[i],rgi);
    }
    last_update_idl = now;

    int sender_next_vertex = *it; it++;
    //printf("%d]\n",sender_next_vertex);
    
    // interrupt path if moving to the same target node
    if (sender_next_vertex == next_vertex) { // two robots are going to the same node
        ROS_INFO("Robots %d and %d are both going to vertex %d",ID_ROBOT,id_sender,next_vertex);
        ROS_INFO("Robot %d: STOP and choose another target",ID_ROBOT);
        // change my destination
        cancelGoal(); // stop the current behavior
        current_vertex = next_vertex; // simulate that the goal vertex has been reached (not sent to the monitor)
        next_vertex = compute_next_vertex(); // compute next vertex (will be different from current vertex)
        sendGoal(next_vertex);
    }
}

int main(int argc, char** argv) {
  
    DTAGreedy_Agent agent;
    agent.init(argc,argv);
    agent.run();

    return 0; 
}
