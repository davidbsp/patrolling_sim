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

#include <stdlib.h>
#include <sstream>
#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Int8MultiArray.h>

#include "globalvars.h"
#include "interferences.h"

void send_goal_result (uint current_vertex, uint next_vertex) { //goal and intention joint together
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

/*void send_intention (uint vertex){
    
    //printf("Send Intention %u [Vertex] to other robots: Robot %d\n",vertex,ID_ROBOT); 
    
    geometry_msgs::PointStamped msg;    
    char id_robot_str[3];
    sprintf(id_robot_str, "%d", ID_ROBOT);  //integer to array
        
    msg.header.frame_id = id_robot_str;
    msg.point.x = float (vertex);
    msg.point.y = 2.0;      //interf=2.0 identifies intention
    msg.point.z = 0.0;

    results_pub.publish(msg);
    ros::spinOnce();
}*/

void backup(){
	
	while (backUpCounter<=100){
	
	if(backUpCounter==0){
		ROS_INFO("The wall is too close! I need to do some backing up...");
		// Move the robot back...
		geometry_msgs::Twist cmd_vel;
		cmd_vel.linear.x = -0.1;
		cmd_vel.angular.z = 0.0;
		cmd_vel_pub.publish(cmd_vel);
	}
			
	if(backUpCounter==40){
		// Turn the robot around...
		geometry_msgs::Twist cmd_vel;
		cmd_vel.linear.x = 0.0;
		cmd_vel.angular.z = 0.5;
		cmd_vel_pub.publish(cmd_vel);
	}
			
	if(backUpCounter==100){
		// Stop the robot...
		geometry_msgs::Twist cmd_vel;
		cmd_vel.linear.x = 0.0;
		cmd_vel.angular.z = 0.0;
		cmd_vel_pub.publish(cmd_vel);
			
		ROS_INFO("Done backing up, now on with my life!");		
	}

	backUpCounter++;
	}
	
}

void resultsCB(const std_msgs::Int8MultiArray::ConstPtr& msg) { 

    std::vector<signed char>::const_iterator it = msg->data.begin();    

    int p1 = *it; //data[0]
    ++it;
    int p2 = *it; //data[1]
    ++it;
    int p3 = *it; //data[2]
    ++it;
    int p4 = *it; //data[2]
    ++it;  
    
      if(initialize==true && p1==-1 && p2==p3 && p3 == p4 && p4 ==0){	//"-1,0,0,0" (BEGINNING)
	  ROS_INFO("Let's Patrol!\n");
	  initialize = false;
      }
	
      if(initialize==false && p1==-1 && p2==1 && p3 == p4 && p4 ==0){	//"-1,1,0,0" (END)
	   ROS_INFO("The simulation is over. Let's leave");
	   end_simulation = true;	  
      }    

      //received vertex and intention from other robot
      if(initialize==false && p1>-1 && p2>-1 && p3>-1 && p4==0){	//ID,vertex,intention,0

	if (p1 != ID_ROBOT){ //protection
	  robot_arrived = p1;
	  vertex_arrived = p2;
	  arrived = true;
	  
	  //this will only be used by SEBS:
	  robot_intention = p1;
	  vertex_intention = p3;
	  intention = true;
	  
	}	
      }
      	 
      ros::spinOnce();
  
}

/*void resultsCB(const geometry_msgs::PointStamped::ConstPtr& msg) { //
	
	if(initialize){	//em espera que o monitor a desbloquei e passe initialize para false
		
// 		printf("Is it the monitor? frame_id = %s (%f,%f,%f)\n",msg->header.frame_id.c_str(),msg->point.x,msg->point.y,msg->point.z);
		
		if (msg->header.frame_id == "monitor"){
			printf("Let's Patrol!\n");
			initialize = false;
		}
	}else {
		if (msg->header.frame_id == "monitor" && msg->point.x == 1.0 && msg->point.y == 1.0 && msg->point.z == 1.0){
			printf("The simulation is over. Let's leave\n");
			end_simulation = true;
		}		
	}
}

void resultsCB_GBS(const geometry_msgs::PointStamped::ConstPtr& msg) {
    
    if(initialize){ //em espera que o monitor a desbloquei e passe initialize para false
        
//      printf("Is it the monitor? frame_id = %s (%f,%f,%f)\n",msg->header.frame_id.c_str(),msg->point.x,msg->point.y,msg->point.z);
        
        if (msg->header.frame_id == "monitor"){
            printf("Let's Patrol!\n");
            initialize = false;
        }
        
    }else {
      
        if (msg->header.frame_id == "monitor" && msg->point.x == 1.0 && msg->point.y == 1.0 && msg->point.z == 1.0){
            printf("The simulation is over. Let's leave\n");
            end_simulation = true;
        
          
        }else if (msg->header.frame_id != "monitor")    {   //Robot reached Goal (including itself!!!)
        
          double interf = msg->point.y;
          double init = msg->point.z;
          
          robot_arrived = atoi( msg->header.frame_id.c_str() ); 
          vertex_arrived = (int)msg->point.x;
            
            if (interf == 0.0 && init == 0.0){            
              if (robot_arrived != ID_ROBOT){   //ignorar msgs do proprio robô:
                arrived = true;                       
                //printf("Robot %d reached Goal %d.\n", robot_arrived, vertex_arrived);
              }
            }
        }
    }
}


void resultsCB_SEBS(const geometry_msgs::PointStamped::ConstPtr& msg) {
    
    if(initialize){ //em espera que o monitor a desbloquei e passe initialize para false
        
//      printf("Is it the monitor? frame_id = %s (%f,%f,%f)\n",msg->header.frame_id.c_str(),msg->point.x,msg->point.y,msg->point.z);
        
        if (msg->header.frame_id == "monitor"){
            printf("Let's Patrol!\n");
            initialize = false;
        }
        
    }else {
      
        if (msg->header.frame_id == "monitor" && msg->point.x == 1.0 && msg->point.y == 1.0 && msg->point.z == 1.0){
            printf("The simulation is over. Let's leave\n");
            end_simulation = true;
        
          
        }else if (msg->header.frame_id != "monitor")    {   //Robot reached Goal or is sending intention (including itself!!!)  
          
          double vert = msg->point.x;
          double interf = msg->point.y;
          double init = msg->point.z;
          
          if (interf == 0.0 && init == 0.0){              
              if (robot_arrived != ID_ROBOT){   //ignorar msgs do proprio robô:

                robot_arrived = atoi( msg->header.frame_id.c_str() ); 
                vertex_arrived = (int)vert;
                arrived = true;                       
                //printf("Robot %d reached Goal %d.\n", robot_arrived, vertex_arrived);
              } 
           } 
           else if(interf == 2.0 && init == 0.0) {     //ROBOT SENDING AN INTENTION
                robot_intention = atoi (msg->header.frame_id.c_str());
                vertex_intention = (int)vert;
                intention = true;
                //printf("Robot %d intends to go to vertex %d.\n", robot_intention, vertex_intention);
           }          
            
        }
    }
}*/

void initialize_node (){ //ID,-1-1,ID
  
	ROS_INFO("Initialize Node: Robot %d",ID_ROBOT);	
	
	std_msgs::Int8MultiArray msg;	
	msg.data.clear();
	msg.data.push_back(ID_ROBOT);
	msg.data.push_back(-1);
	msg.data.push_back(-1);
	msg.data.push_back(ID_ROBOT);	
	
	int count = 0;
	
	//ATENÇÃO ao PUBLICADOR!
	ros::Rate loop_rate(0.5); //meio segundo
	
	while (count<3){ //send activation msg 3times
		results_pub.publish(msg);
		//ROS_INFO("publiquei msg: %s\n", msg.data.c_str());
		ros::spinOnce();
		loop_rate.sleep();
		count++;
	}
}

// GOAL

void goalDoneCallback(const actionlib::SimpleClientGoalState &state, const move_base_msgs::MoveBaseResultConstPtr &result){ //goal terminado (completo ou cancelado)
//  ROS_INFO("Goal is complete (suceeded, aborted or cancelled).");
    // If the goal succeeded send a new one!
    //if(state.state_ == actionlib::SimpleClientGoalState::SUCCEEDED) sendNewGoal = true;
    // If it was aborted time to back up!
    //if(state.state_ == actionlib::SimpleClientGoalState::ABORTED) needToBackUp = true;    
    
    if(state.state_ == actionlib::SimpleClientGoalState::SUCCEEDED){
        ROS_INFO("SUCCESS");
        goal_complete = true;
        //send_goal_result (next_vertex); //goal and intention were merged
        
    }else{
        ROS_INFO("CANCELLED or ABORTED...BACKUP & Resend Goal!");   //tentar voltar a enviar goal..
        backUpCounter = 0;
        backup();
        ResendGoal = true;
    }
}

void goalActiveCallback(){  //enquanto o robot esta a andar para o goal...
    goal_complete = false;
//      ROS_INFO("Goal is active.");
}

void goalFeedbackCallback(const move_base_msgs::MoveBaseFeedbackConstPtr &feedback){	//publicar posições

// 	printf("ID_ROBOT = %d [feedback callback]\n", ID_ROBOT);
	
	//Publish Position to common node:
	nav_msgs::Odometry msg;	
	
	int idx = ID_ROBOT;
		
	if (ID_ROBOT <= -1){
		msg.header.frame_id = "odom";	 //identificador do robot q publicou
		idx = 0;
	}else{
		char string[20];
		/*strcpy (string,"robot_"); 
		char id[3];
        sprintf(id,"%d",ID_ROBOT); 
		// itoa(ID_ROBOT, id, 10);  
		strcat(string,id);
		strcat(string,"/odom"); //string = "robot_X/odom" 	
		//strcpy( msg.header.frame_id, string );
        */
        sprintf(string,"robot_%d/odom",ID_ROBOT);
		msg.header.frame_id = string;
	}
	
	msg.pose.pose.position.x = xPos[idx]; //send odometry.x
	msg.pose.pose.position.y = yPos[idx]; //send odometry.y
	
	odom_pub.publish(msg);
	ros::spinOnce();
	
   	//ROS_INFO("[POSITION PUBLISHED]: ID = %s, X = %f, Y = %f", msg.header.frame_id.c_str(), xPos[idx], yPos[idx]);
// 	printf("ID_ROBOT = %d\n", ID_ROBOT);
// 	printf("TEAMSIZE = %d\n", TEAMSIZE);	
	interference = check_interference(ID_ROBOT);	
}



void sendGoal(MoveBaseClient &ac, double target_x, double target_y) 
{
    //Define Goal:
    move_base_msgs::MoveBaseGoal goal;
    //Send the goal to the robot (Global Map)
    geometry_msgs::Quaternion angle_quat = tf::createQuaternionMsgFromYaw(0.0);     
    goal.target_pose.header.frame_id = "map"; 
    goal.target_pose.header.stamp = ros::Time::now();    
    goal.target_pose.pose.position.x = target_x; // vertex_web[current_vertex].x;
    goal.target_pose.pose.position.y = target_y; // vertex_web[current_vertex].y;  
    goal.target_pose.pose.orientation = angle_quat; //alpha -> orientação  (queria optimizar este parametro -> através da direcção do vizinho!)
    ac.sendGoal(goal, boost::bind(&goalDoneCallback, _1, _2), boost::bind(&goalActiveCallback), boost::bind(&goalFeedbackCallback, _1));  
}

