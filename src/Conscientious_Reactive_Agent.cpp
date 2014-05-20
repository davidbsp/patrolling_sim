/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2011, ISR University of Coimbra.
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
* Author: David Portugal, 2011
*********************************************************************/
#include <sstream>
#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>

#include "PatrolAgent.h"



class Conscientious_Reactive_Agent: public PatrolAgent {
    
public:
    virtual int compute_next_vertex();
    virtual void send_results();
    virtual void receive_results();    
};



int Conscientious_Reactive_Agent::compute_next_vertex() {

  //number of neighbors of current vertex (number of existing possibilites)
  uint num_neighs = vertex_web[current_vertex].num_neigh;
  uint next_vertex;
  
  if (num_neighs > 1){
    
    double decision_table [num_neighs];
    uint neighbors [num_neighs];
    uint possibilities[num_neighs];
     
    uint i, hits=0;
    double max_idleness= -1;
    
    for (i=0; i<num_neighs; i++){
        neighbors[i] = vertex_web[current_vertex].id_neigh[i];        //neighbors table
        decision_table[i] = instantaneous_idleness [ neighbors[i] ];  //corresponding idleness table
        //printf("   --- vertex %u -> idleness %.1f\n",neighbors[i],decision_table[i]);
        
        //choose the one with maximum idleness:
        if (decision_table[i] > max_idleness){
            max_idleness = decision_table[i];       //maximum idleness
            hits=0;
            possibilities[hits] = neighbors[i];    
        } 
        else if(decision_table[i] == max_idleness) {
            hits ++;
            possibilities[hits] = neighbors[i];
        }
    }      
      
    if(hits>0){ //more than one possibility (choose at random)
      srand ( time(NULL) );
      i = rand() % (hits+1) + 0;    //0, ... ,hits
    
      //printf("rand integer = %d\n", i);
      next_vertex = possibilities [i];      // random vertex with higher idleness
        
      }else{
        next_vertex = possibilities[hits];  //vertex with higher idleness
      }
    
    }else{
        next_vertex = vertex_web[current_vertex].id_neigh[0]; //only one possibility
    }
  
    ROS_INFO("Conscientious_Reactive choice: %d",next_vertex);
    return next_vertex;
}


// FIXME Needed???
void Conscientious_Reactive_Agent::send_results() {
    //goal: [ID,vertex,intention,0]

    std_msgs::Int8MultiArray msg;   
    msg.data.clear();
    msg.data.push_back(ID_ROBOT);
    msg.data.push_back(current_vertex);
    msg.data.push_back(next_vertex);
    msg.data.push_back(0);
    
    results_pub.publish(msg);   
    ros::spinOnce();    
}

// FIXME Needed???
void Conscientious_Reactive_Agent::receive_results() {
    //goal: [ID,vertex,intention,0]

    //received vertex and intention from other robot
    if(initialize==false && vresults[0]>-1 && vresults[1]>-1 && vresults[2]>-1 && vresults[3]==0){    //ID,vertex,intention,0

        if (vresults[0] != ID_ROBOT){ //protection
            robot_arrived = vresults[0];
            vertex_arrived = vresults[1];
            arrived = true;
            
            //this will only be used by SEBS:
            robot_intention = vresults[0];
            vertex_intention = vresults[2];
            intention = true;
        }   
    } 
}


int main(int argc, char** argv) {
  
    Conscientious_Reactive_Agent agent;
    agent.init(argc,argv);
    agent.run();

    return 0; 
}

