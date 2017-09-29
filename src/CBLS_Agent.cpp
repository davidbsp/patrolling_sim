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
* Author: David Portugal (2011-2014), and Luca Iocchi (2014-2016)
*********************************************************************/

#include <sstream>
#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>

#include "PatrolAgent.h"
#include "getgraph.h"
#include "algorithms.h"


using namespace std;

class CBLS_Agent: public PatrolAgent {

private:

  int NUMBER_OF_ROBOTS;
  int *tab_intention;
  uint *node_count;
  bool arrived;
  uint vertex_arrived;
  int robot_arrived;
  bool intention;
  uint vertex_intention;
  int robot_intention;  
  int number_of_edges;
  reinforcement_learning RL;
  long long int decision_number;
  double now;
  double *avg_idleness;  // local idleness
  double *cur_avg_idleness;  // local idleness
  double *real_histogram; 
  double *histogram;
  uint *source;
  uint *destination;
  uint hist_dimension;
      
public:
    virtual void init(int argc, char** argv);
    virtual int compute_next_vertex();
    virtual void processEvents();
    virtual void send_results();
    virtual void receive_results();    
    virtual void onGoalComplete();    
};


void CBLS_Agent::init(int argc, char** argv) {
   
  PatrolAgent::init(argc,argv);
  
  NUMBER_OF_ROBOTS = atoi(argv[3]);
  uint number_of_edges = GetNumberEdges(vertex_web, dimension);  
    
  //INITIALIZE tab_intention:
  int i;
  
  tab_intention = new int[NUMBER_OF_ROBOTS];
  for (i=0; i<NUMBER_OF_ROBOTS; i++){
    tab_intention[i] = -1;
  }
  
  //INITIALIZE node_count:
  node_count = new uint[dimension];
  for (i=0; i<dimension; i++){
    node_count[i] = 0;
  }  
  
  hist_dimension = 2*number_of_edges; //directed edges, i.e.: arcs
  
  source = new uint [hist_dimension];
  destination = new uint [hist_dimension];

  create_source_and_dest_tables(vertex_web, source, destination, dimension);
  
  real_histogram = new double[hist_dimension];
  histogram = new double[hist_dimension];	 // between 0 and 1

  for (i=0; i<hist_dimension; i++){
    real_histogram[i] = 1.0;
    histogram[i] = 1.0 / (double) hist_dimension;
  } 
  
  //INITIALIZE tables:
  double time_zero = ros::Time::now().toSec();  

  avg_idleness = new double[dimension];	//closed avg
  cur_avg_idleness = new double[dimension];	//avg + inst
   
  for(i=0;i<dimension;i++){ 
    instantaneous_idleness[i]= 0.0;
    avg_idleness[i] = 0.0;
    cur_avg_idleness[i]=0.0;
    last_visit[i]= time_zero; 
    
    if(i==current_vertex){
      last_visit[i]= time_zero + 0.1; //Avoids getting back immediately at the initial vertex
      node_count[i]= 1;
    }
  }
  
  decision_number = 0;  
  
}

void CBLS_Agent::onGoalComplete() {
  
    if( (next_vertex>-1) && (current_vertex != next_vertex) ){
      
    		/** PUNISH & REWARD -- BEFORE **/ 
		//Update Idleness Table:
		now = ros::Time::now().toSec();
			
		for(int i=0; i<dimension; i++){
			if (i == next_vertex){
				last_visit[i] = now;
				node_count[i]++;
				avg_idleness[i] = ( avg_idleness[i] * (double) (node_count [i] - 1) + instantaneous_idleness [i] ) / ( (double) node_count [i] );		  		    
			}	
			instantaneous_idleness[i]= now - last_visit[i];  //ou seja: Zero para o next_vertex			
			//ROS_INFO("inst_idleness[%d] = %f", i, instantaneous_idleness[i]);
			
			//Update Curr Avg Idleness Table:
			cur_avg_idleness [i] = ( avg_idleness [i] * (double) (node_count [i]) + instantaneous_idleness [i] ) / ( (double) node_count [i] + 1 );
		}
		
		/** PUNISH & REWARD -- NOW **/
                int value = ID_ROBOT;
                if (value==-1){value=0;}
                
		update_likelihood_new(RL, node_count, instantaneous_idleness, dimension, real_histogram, source, destination, hist_dimension, vertex_web, value);
		normalize_histogram(real_histogram, histogram, hist_dimension);	
		  
		/*if(decision_number%50 == 0){ //write histogram each 50 iterations 
		  write_histogram_to_file (vertex_web, real_histogram, histogram, source, destination, hist_dimension,decision_number,value);
		}*/
		  
		decision_number++;
		//ROS_INFO("decision_number = %lld", decision_number);		
		  
		current_vertex = next_vertex;		  
    }
    
	/** *************CALL LEARNING FUNCTION *****************/
	next_vertex = compute_next_vertex();
	
	if(next_vertex == -1){
	 ROS_ERROR("ABORT (learning_algorithm: next_vertex = -1)");
	 exit(-1);
	}	
	/** *****************************************************/  
	
	
	/** David Portugal: 23 Dec. 2015 -- changed this behavior. 
	 *  The robot no longer stays in the same place if all neighbor vertices are occupied**/
	
	 //if (current_vertex==next_vertex){
	 //  goal_complete = true; //do not try to go there!
	 //  //Stay in the same place to avoid interference
	   
	 //}else{
	    
	  //send_info(current_vertex, next_vertex);
	  send_goal_reached(); // Send TARGET to monitor
	  send_results();  // Algorithm specific function
	  
	  ROS_INFO("Sending goal - Vertex %d (%f,%f)\n", next_vertex, vertex_web[next_vertex].x, vertex_web[next_vertex].y);
	  //sendGoal(vertex_web[next_vertex].x, vertex_web[next_vertex].y);  
	  sendGoal(next_vertex);  // send to move_base
	  
	  goal_complete = false;    	  
	 //}	
  
}

// Executed at any cycle when goal is not reached
void CBLS_Agent::processEvents() {
    
    if (arrived && NUMBER_OF_ROBOTS>1){ //a different robot arrived at a vertex: update idleness table and keep track of last vertices positions of other robots.

        //ROS_INFO("Robot %d reached Goal %d.\n", robot_arrived, vertex_arrived);    

        //Update Idleness Table:
        now = ros::Time::now().toSec();
                
        for(int i=0; i<dimension; i++){
            if (i == vertex_arrived){
                //actualizar last_visit[dimension]
                last_visit[vertex_arrived] = now;   
			  node_count[vertex_arrived]++;
			  avg_idleness[i] = ( avg_idleness[i] * (double) (node_count [i] - 1) + instantaneous_idleness [i] ) / ( (double) node_count [i] );		
            }           
            //actualizar instantaneous_idleness[dimension]
            instantaneous_idleness[i] = now - last_visit[i];      
	    cur_avg_idleness [i] = ( avg_idleness [i] * (double) (node_count [i]) + instantaneous_idleness [i] ) / ( (double) node_count [i] + 1 );		  	  		  	    
	    //ROS_INFO("idleness[%d] = %f", i, instantaneous_idleness[i]);
        }     
        
        arrived = false;
    }

    if (intention && NUMBER_OF_ROBOTS>1) {    
        tab_intention[robot_intention] = vertex_intention;
        //printf("tab_intention[ID=%d]=%d\n",robot_intention,tab_intention[robot_intention]);
        intention = false;
    }
    // ros::spinOnce();   

}

int CBLS_Agent::compute_next_vertex() {
    int value = ID_ROBOT;
    if (value==-1){value=0;}
    return learning_algorithm (current_vertex, vertex_web, instantaneous_idleness, cur_avg_idleness, tab_intention, histogram, source, destination, hist_dimension, TEAMSIZE, value, node_count, RL);
}


void CBLS_Agent::send_results() {  
    int value = ID_ROBOT;
    if (value==-1){value=0;}
    // [ID,msg_type,vertex,intention]
    std_msgs::Int16MultiArray msg;   
    msg.data.clear();
    msg.data.push_back(value);
    msg.data.push_back(CBLS_MSG_TYPE);
    msg.data.push_back(current_vertex);
    msg.data.push_back(next_vertex);    
    do_send_message(msg);
}

void CBLS_Agent::receive_results() {
  
    std::vector<int>::const_iterator it = vresults.begin();
    int id_sender = *it; it++;
    int msg_type = *it; it++;
    
    int value = ID_ROBOT;
    if (value==-1){value=0;}
    
  	if ((id_sender==value) || (msg_type!=CBLS_MSG_TYPE)) 
    	return;
        
    robot_arrived = vresults[0];
    vertex_arrived = vresults[2];
    arrived = true;
    robot_intention = vresults[0];
    vertex_intention = vresults[3];
    intention = true;
}

int main(int argc, char** argv) {

    CBLS_Agent agent;
    agent.init(argc,argv);    
    agent.run();

    return 0; 
}


