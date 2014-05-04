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

#include "getgraph.h"
#include "algorithms.h"

#include "globalvars.h"
#include "interferences.h"
#include "actions.h"


int main(int argc, char** argv){	//pass the .graph file to open
 /*
  argc=3
  argv[0]=/.../patrolling_sim/bin/MSP
  argv[1]=__name:=XXXXXX
  argv[2]=maps/1r-5-map.graph
  argv[3]=MSP/1r5-map_1_0
  argv[4]=ID_ROBOT
  */
  
  //More than One robot (ID between 0 and 99)
  if ( atoi(argv[4])>NUM_MAX_ROBOTS || atoi(argv[4])<-1 ){
    ROS_INFO("The Robot's ID must be an integer number between 0 an 99"); //max 100 robots 
    return 0;
  }else{
    ID_ROBOT = atoi(argv[4]); 
    //printf("ID_ROBOT = %d\n",ID_ROBOT); //-1 in the case there is only 1 robot.
  }

  uint i = strlen( argv[2] );
  char graph_file[i];
  strcpy (graph_file,argv[2]);
  
  //Check Graph Dimension:
  uint dimension = GetGraphDimension(graph_file);
  
  //Create Structure to save the Graph Info;
  vertex vertex_web[dimension];
  
  //Get the Graph info from the Graph File
  GetGraphInfo(vertex_web, dimension, graph_file);
  
  uint j;
  
  
  i = strlen( argv[3] );
  char msp_file[i];
  strcpy (msp_file,argv[3]);
  
//   printf("MSP_FILE = %s\n",msp_file);    
  
  
  /* Output Graph Data */
  for (i=0;i<dimension;i++){
    printf ("ID= %u\n", vertex_web[i].id);
    printf ("X= %f, Y= %f\n", vertex_web[i].x, vertex_web[i].y);
    printf ("#Neigh= %u\n", vertex_web[i].num_neigh);
	
    for (j=0;j<vertex_web[i].num_neigh; j++){
      printf("\tID = %u, DIR = %s, COST = %u\n", vertex_web[i].id_neigh[j], vertex_web[i].dir[j], vertex_web[i].cost[j]);
    }
    
    printf("\n");	
  }
  
  
  
  /* Define Starting Vertex/Position (Launch File Parameters) */

  ros::init(argc, argv, "msp");
  ros::NodeHandle nh;
  double initial_x, initial_y;
  
  XmlRpc::XmlRpcValue list;
  nh.getParam("initial_pos", list);
  ROS_ASSERT(list.getType() == XmlRpc::XmlRpcValue::TypeArray);
  
  int value = ID_ROBOT;
  if (value == -1){value = 0;}
  
  ROS_ASSERT(list[2*value].getType() == XmlRpc::XmlRpcValue::TypeDouble);
  initial_x = static_cast<double>(list[2*value]);
  
  ROS_ASSERT(list[2*value+1].getType() == XmlRpc::XmlRpcValue::TypeDouble);
  initial_y = static_cast<double>(list[2*value+1]);
 
//    printf("x=%f, y=%f\n", initial_x, initial_y);
  uint current_vertex = IdentifyVertex(vertex_web, dimension, initial_x, initial_y);
//    printf("v=%d\n",current_vertex);
  
  //Publicar dados de "odom" para nó de posições
  odom_pub = nh.advertise<nav_msgs::Odometry>("positions", 1); //only concerned about the most recent
	
  //Subscrever posições de outros robots
  odom_sub = nh.subscribe("positions", 10, positionsCB);  
  
  char string[20];
  char string2[20];
  
  if(ID_ROBOT==-1){ 
    strcpy (string,"odom"); //string = "odom"
    strcpy (string2,"cmd_vel"); //string = "cmd_vel"
    TEAMSIZE = 1;
  }else{ 
    strcpy (string,"robot_"); 
    strcpy (string2,"robot_"); 
    char id[3];
    itoa(ID_ROBOT, id, 10);  
    strcat(string,id);
    strcat(string2,id);
    strcat(string,"/odom"); //string = "robot_X/odom"  
    strcat(string2,"/cmd_vel"); //string = "robot_X/cmd_vel"
    TEAMSIZE = ID_ROBOT + 1;
  }	  
  
//   printf("string de publicação da odometria: %s\n",string);
  
   //Cmd_vel to backup:
   cmd_vel_pub  = nh.advertise<geometry_msgs::Twist>(string2, 1);  
  
  //Subscrever para obter dados de "odom" do robot corrente
  ros::Subscriber sub;
  sub = nh.subscribe(string, 1, odomCB); //size of the buffer = 1 (?)
  ros::spinOnce();    
    
  
  // Define Goal
  
  if(ID_ROBOT==-1){ 
    strcpy (string,"move_base"); //string = "move_base"  
  }else{ 
    strcpy (string,"robot_"); 
    char id[3];
    itoa(ID_ROBOT, id, 10);  
    strcat(string,id);
    strcat(string,"/move_base"); //string = "robot_X/move_base"  
  }
  
  //printf("string = %s",string);
  MoveBaseClient ac(string, true); 
  
  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
     ROS_INFO("Waiting for the move_base action server to come up");
  }
  
  //Publicar dados para "results"
  results_pub = nh.advertise<std_msgs::Int8MultiArray>("results", 100);
  results_sub = nh.subscribe("results", 10, resultsCB); //Subscrever "results" vindo dos robots
  
  initialize_node(); //dizer q está vivo
  ros::Rate loop_rate(1); //1 segundo
  
  /* Wait until all nodes are ready.. */
  while(initialize){
	ros::spinOnce();
	loop_rate.sleep();
  }  

  /* Set up listener for global coordinates of robots */
  listener = new tf::TransformListener();

 
  //Check Route Dimension:
  uint route_dimension = get_MSP_dimension(msp_file);
  
  //Create Structure to save the Route Info;
  uint route [route_dimension];
  
  //Get the Route info from the MSP Route File
  get_MSP_route (route, route_dimension, msp_file);
  
  printf("\nFinal Path: ");
  for(i=0; i<route_dimension; i++){
	if(i==route_dimension-1){ printf("%i\n", route[i]); }else{ printf("%i, ", route[i]); }
  }
  printf("Number of elements = %i\n", route_dimension);  
  
  if(route_dimension>1){ i=1; next_vertex = route[i]; }  
  
  interference = false;
  ResendGoal = false;
  goal_complete = true;
    
  
  while(ros::ok()) {
	  
	if(goal_complete){  
	  
		/** SEND GOAL (REACHED) AND INTENTION **/
		send_goal_result (current_vertex, next_vertex);	  
	  
		//printf("Move Robot to Vertex %d (%f,%f)\n", next_vertex, vertex_web[next_vertex].x, vertex_web[next_vertex].y);
		ROS_INFO("Sending goal - Vertex %d (%f,%f)", next_vertex, vertex_web[next_vertex].x, vertex_web[next_vertex].y);
        sendGoal(ac,vertex_web[next_vertex].x, vertex_web[next_vertex].y);
		//goalvertex = next_vertex;
		
		current_vertex = next_vertex;
		i++;
		if ( i>=route_dimension ){ i=1;}
		next_vertex = route[i];    
		goal_complete = false;
	}else{
		if (interference){
			do_interference_behavior();		
		}	    
		
		if(ResendGoal){
            ROS_INFO("Resending goal - Vertex %d (%f,%f)\n", current_vertex, vertex_web[current_vertex].x, vertex_web[current_vertex].y);
            sendGoal(ac,vertex_web[current_vertex].x, vertex_web[current_vertex].y);
			//goalvertex = current_vertex;
			ResendGoal = false; //para nao voltar a entrar (envia goal so uma vez)
		}  
		
		if(end_simulation){
			return 0;
		}		
	}
	
    ros::Duration delay = ros::Duration(0.1);
    delay.sleep();

  } // while ros.ok
  
  return 0; 
}
