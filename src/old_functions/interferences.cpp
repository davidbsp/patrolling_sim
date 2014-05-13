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
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Int8MultiArray.h>

#include "globalvars.h"

void getRobotPose(int robotid, float &x, float &y, float &theta) {
    
    if (listener==NULL) {
        ROS_ERROR("TF listener null");
        return;
    }
    
    std::stringstream ss; ss << "robot_" << robotid;
    std::string robotname = ss.str();
    std::string sframe = "/map";				//Patch David Portugal: Remember that the global map frame is "/map"
    std::string dframe = "/" + robotname + "/base_link";
    tf::StampedTransform transform;

    try {
        listener->waitForTransform(sframe, dframe, ros::Time(0), ros::Duration(3));
        listener->lookupTransform(sframe, dframe, ros::Time(0), transform);
    }
    catch(tf::TransformException ex) {
        ROS_ERROR("Cannot transform from %s to %s\n",sframe.c_str(),dframe.c_str());
        ROS_ERROR("%s", ex.what());
    }

    x = transform.getOrigin().x();
    y = transform.getOrigin().y();
    theta = tf::getYaw(transform.getRotation());
    // printf("Robot %d pose : %.1f %.1f \n",robotid,x,y);
}


void odomCB(const nav_msgs::Odometry::ConstPtr& msg) { //colocar propria posicao na tabela
	
// 	printf("Colocar Propria posição na tabela, ID_ROBOT = %d\n",ID_ROBOT);
	int idx = ID_ROBOT;
	
	if (ID_ROBOT<=-1){
		idx = 0;
	}
	
	float x,y,th;
	getRobotPose(idx,x,y,th);
	
	xPos[idx]=x; // msg->pose.pose.position.x;
	yPos[idx]=y; // msg->pose.pose.position.y;
	
// 	printf("Posicao colocada em Pos[%d]\n",idx);
}

void positionsCB(const nav_msgs::Odometry::ConstPtr& msg) {	//construir tabelas de posições
        
//     printf("Construir tabela de posicoes (receber posicoes), ID_ROBOT = %d\n",ID_ROBOT);    
        
    char id[20]; //identificador do robot q enviou a msg d posição...
    strcpy( id, msg->header.frame_id.c_str() );
    //int stamp = msg->header.seq;
//     printf("robot q mandou msg = %s\n", id);
    
    // Build Positions Table
    
    if (ID_ROBOT>-1){
	//verify id "XX" of robot: (string: "robot_XX/odom")
	
	char str_idx[4];
	uint i;
	
	for (i=6; i<10; i++){
		if (id[i]=='/'){
			str_idx[i-6] = '\0';
			break;
		}else{
			str_idx[i-6] = id[i];
		}
	}
	
	int idx = atoi (str_idx);
// 	printf("id robot q mandou msg = %d\n",idx);
	
	if (idx >= TEAMSIZE && TEAMSIZE <= NUM_MAX_ROBOTS){
		//update teamsize:
		TEAMSIZE = idx+1;
	}
	
	if (ID_ROBOT != idx){  //Ignore own positions	
		xPos[idx]=msg->pose.pose.position.x;
		yPos[idx]=msg->pose.pose.position.y;		
	}	
//  	printf ("Position Table:\n frame.id = %s\n id_robot = %d\n xPos[%d] = %f\n yPos[%d] = %f\n\n", id, idx, idx, xPos[idx], idx, yPos[idx] );	    
    }
}



void send_interference(int ID_ROBOT, ros::Publisher &results_pub){
//interference: [ID,-1,-2,1]

	printf("Send Interference: Robot %d\n",ID_ROBOT);	
	
	std_msgs::Int8MultiArray msg;	
	msg.data.clear();
	msg.data.push_back(ID_ROBOT);
	msg.data.push_back(-1);
	msg.data.push_back(-2);
	msg.data.push_back(1);
	
	results_pub.publish(msg);	
	ros::spinOnce();
}


bool check_interference (int ID_ROBOT){	//verificar se os robots estao proximos
	
	int i;
	double dist_quad;
	
	/* Poderei usar TEAMSIZE para afinar */
	for (i=0; i<ID_ROBOT; i++){	//percorrer vizinhos (assim asseguro q cada interferencia é so encontrada 1 vez)
		
		dist_quad = (xPos[i] - xPos[ID_ROBOT])*(xPos[i] - xPos[ID_ROBOT]) + (yPos[i] - yPos[ID_ROBOT])*(yPos[i] - yPos[ID_ROBOT]);
		
		if (dist_quad <= /*sqrt*/4){	//robots are 2 meter or less apart
// 			ROS_INFO("Feedback: Robots are very close. INTERFERENCE! Dist_Quad = %f", dist_quad);
			return true;
		}		
	}
	return false;
	
}

void do_interference_behavior()
{
        // Stop the robot..         
        ROS_INFO("Interference detected!\n");   
        send_interference(ID_ROBOT,results_pub);

        //get own "odom" positions...
        ros::spinOnce();        
                    
        //Waiting until conflict is solved...
        while(interference){
            interference = check_interference(ID_ROBOT);
            if (goal_complete || ResendGoal){
                interference = false;
            }
        }
            // se saiu é pq interference = false            
}
