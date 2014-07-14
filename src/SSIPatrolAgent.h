#include <sstream>
#include <string>
#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Int16MultiArray.h>
#include <algorithm>
#include <stdio.h>

#include "PatrolAgent.h"
#include "algorithms.h"
#include "config.h"


#define CONFIG_FILENAME "params/DTA/DTASSI.params"

#define BIG_NUMBER 10000

//a tuple <robotId,bidValue>
typedef struct bid_tuple {
	double bidValue;
	int robotId;
} bid_tuple;


class SSIPatrolAgent: public PatrolAgent {

protected:

	//true if I am selecting the first vertex to go to	
    bool first_vertex; 

	//the goal I will go when I reach the current goal 
    //required to avoid that the robot waits idle after completing each goal
	int next_next_vertex;    


    double *global_instantaneous_idleness;  // global estimated idleness
    double last_update_idl;

    //time of task requests arrived for each room. A vector of integer, where taskRequest[i] 
    //is the time at which this robot received a task request for location i  
    int* taskRequests;

    //tasks this robot is responsible for. A boolean vector, where the value of index i is 1 if I have to visit that location 
    bool* tasks; 

    //vertices that has been selected within the same optimization loop but for which the robot did not win the auction
    bool* selected_vertices;
    
    //waiting time for collecting all bids from other robots
    double timeout;
    
    //threshold on idleness for considering a location just visited 
    double threshold; 		 	    
  
    //bids received from other robots. A vector indexed by locations. 
    //bids[i] = <robotId, bidValue>, where bidValue is the minimum bid received for location i and senderId is the sender
    bid_tuple* bids;

    //parameter to weight Idleness in the util computation
    double theta_idl;

    //parameter to weight navigation cost in the util computation
    double theta_cost;

    //increase the value of a bid when received to avoid switching when there is no real gain, put this to zero to avoid using histeresys 
    //0 -> less interferences, high stddev
    double hist;

    ConfigFile cf;
	
    //allocate an array of bool one for each vertex, set all to false
    bool* create_selected_vertices();

    //set all selected vertices to false, set current_vertex to true (avoid considering current_vertex as next target)
    void reset_selected_vertices(bool* sv);

	//set all selected vertices to true, set neighbouring vertices to false (consider only neighbouring vertices as next target)
	void select_faraway_vertices(bool* selected_vertices, int cv);

    //check if all vertices have been selected
    bool all_selected(bool* sv);
	
    //select the next best vertex (used to select which task should be auctioned);
    //mark vertex that have been selected to avoid selecting them in the same alg step	
    int select_next_vertex(int currv, bool* sv);
 
    //compute minimum path cost considering all tasks (tasks) and the next vertex (nv). 
    //The first room is always the current goal (if any), then rooms are visited in decreasing order of utility. 
    //The path cost is sum of travel cost given the order. 
    virtual double compute_bid(int nv);	

    //force the best bid for dest to be the one from robotId with value bidvalue	
    void force_bid(int nv,double bidvalue,int robotId);

    //update tasks seeting to true only the vertices for which this robot has the current highest bid
    //based on the array bids	
    virtual void update_tasks();

    //announce the intension to go to vertex nv with a bid value of bv
    void send_target(int nv,double bv);

    //computes whether this robot offered the best bid for vertex nv
    //based on the array bids	
    bool best_bid(int nv);

    //return geometric distance from current robot position to vertex
    double compute_distance(int vertex);

    //return path cost from vertex cv to vertex nv 
    double compute_cost(int cv, int nv);	

    virtual void update_bids(int next_vertex, double bid_value, int senderId);

    void send_bid(int nv,double bv);

    void idleness_msg_handler(std::vector<int>::const_iterator it);

    void task_request_msg_handler(std::vector<int>::const_iterator it, int sender_id);

    void bid_msg_handler(std::vector<int>::const_iterator it, int sender_id);

    void wait();	


public:

    SSIPatrolAgent() : cf(CONFIG_FILENAME)
    {}

    virtual void init(int argc, char** argv);
	virtual void onGoalComplete();    
    virtual int compute_next_vertex();
    virtual void send_results();
    virtual void receive_results();    

	int compute_next_vertex(int cv);
    double compute_cost(int vertex);
    double utility(int currentv, int nextv);
    void update_global_idleness();
};

