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

#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

#include "getgraph.h"
#include "algorithms.h"

#define NUM_MAX_ROBOTS 32

typedef unsigned int uint;
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

bool ResendGoal; //Send the same goal again (if goal failed...)
bool interference;
bool goal_complete;
bool initialize = true;
bool end_simulation = false;
int next_vertex = -1;
uint backUpCounter;

int TEAMSIZE;
int ID_ROBOT;

double xPos[NUM_MAX_ROBOTS]; //tabelas de posições (atençao ao index pro caso de 1 so robot)
double yPos[NUM_MAX_ROBOTS]; //tabelas de posições (atençao ao index pro caso de 1 so robot)

ros::Subscriber odom_sub;
ros::Publisher odom_pub;
ros::Subscriber results_sub;
ros::Publisher results_pub;
ros::Publisher cmd_vel_pub;

void send_goal_result (uint vertex){
  /*
  string frame_id	//robot ID
    float64 x		//Goal (X=vertex,0,0)  
*/  	
	printf("Send Vertex %u [Goal] to Results: Robot %d\n",vertex,ID_ROBOT);	
	
	geometry_msgs::PointStamped msg;	
	char id_robot_str[3];
	sprintf(id_robot_str, "%d", ID_ROBOT);	//integer to array
		
	msg.header.frame_id = id_robot_str;
	msg.point.x = float (vertex);
	msg.point.y = 0.0; 
	msg.point.z = 0.0;

	results_pub.publish(msg);
	ros::spinOnce();
}

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

void goalDoneCallback(const actionlib::SimpleClientGoalState &state, const move_base_msgs::MoveBaseResultConstPtr &result){ //goal terminado (completo ou cancelado)
// 	ROS_INFO("Goal is complete (suceeded, aborted or cancelled).");
	// If the goal succeeded send a new one!
	//if(state.state_ == actionlib::SimpleClientGoalState::SUCCEEDED) sendNewGoal = true;
	// If it was aborted time to back up!
	//if(state.state_ == actionlib::SimpleClientGoalState::ABORTED) needToBackUp = true;	
	
	if(state.state_ == actionlib::SimpleClientGoalState::SUCCEEDED){
 		ROS_INFO("SUCCESS");
		goal_complete = true;
		send_goal_result (next_vertex);
		
	}else{
 		ROS_INFO("CANCELLED or ABORTED...BACKUP & Resend Goal!");	//tentar voltar a enviar goal..
		backUpCounter = 0;
 		backup();
		ResendGoal = true;
	}
}

void goalActiveCallback(){	//enquanto o robot esta a andar para o goal...
	goal_complete = false;
//  	ROS_INFO("Goal is active.");
}

void odomCB(const nav_msgs::Odometry::ConstPtr& msg) { //colocar propria posicao na tabela
	
// 	printf("Colocar Propria posição na tabela, ID_ROBOT = %d\n",ID_ROBOT);
	int idx = ID_ROBOT;
	
	if (ID_ROBOT<=-1){
		idx = 0;
	}
	
	xPos[idx]=msg->pose.pose.position.x;
	yPos[idx]=msg->pose.pose.position.y;
	
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

void resultsCB(const geometry_msgs::PointStamped::ConstPtr& msg) { //
	
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

void initialize_node (){
  /*
  string frame_id	//robot ID
    float64 z		//initialization (0,0,Z=ID)  
*/  	
	printf("Initialize Node: Robot %d\n",ID_ROBOT);	
	ros::Rate loop_rate(1); //1 segundo
	
	geometry_msgs::PointStamped msg;	
	
	char id_robot_str[3];
	sprintf(id_robot_str, "%d", ID_ROBOT);	//integer to array
		
	msg.header.frame_id = id_robot_str;
	msg.point.x = 0.0;
	msg.point.y = 0.0; 
	msg.point.z = (float) ID_ROBOT; //Z = ID
	
	int count = 0;
	while (count<2){
		results_pub.publish(msg);
		ros::spinOnce();
		//printf("publiquei msg.\n");
		loop_rate.sleep();
		count++;
	}	
}

void send_interference(){
  /*
  string frame_id	//robot ID
    float64 y		//Interference (0,Y=1,0)  
*/  	
	printf("Send Interference: Robot %d\n",ID_ROBOT);	
	//ros::Rate loop_rate(1); //1 segundo
	
	geometry_msgs::PointStamped msg;	
	char id_robot_str[3];
	sprintf(id_robot_str, "%d", ID_ROBOT);	//integer to array
		
	msg.header.frame_id = id_robot_str;
	msg.point.x = 0.0;
	msg.point.y = 1.0; 
	msg.point.z = 0.0;

	results_pub.publish(msg);
	ros::spinOnce();
}

bool check_interference (void){	//verificar se os robots estao proximos
	
	int i;
	double dist_quad;
	
	/* Poderei usar TEAMSIZE para afinar */
	for (i=0; i<ID_ROBOT; i++){	//percorrer vizinhos (assim asseguro q cada interferencia é so encontrada 1 vez)
		
		dist_quad = (xPos[i] - xPos[ID_ROBOT])*(xPos[i] - xPos[ID_ROBOT]) + (yPos[i] - yPos[ID_ROBOT])*(yPos[i] - yPos[ID_ROBOT]);
		
		if (dist_quad <= /*sqrt*/4){	//robots are 2 meter or less apart
// 			ROS_INFO("Feedback: Robots are very close. INTERFERENCE! Dist_Quad = %f", dist_quad);
			interference = true;
			return interference;
		}		
	}
	
	interference = false;
	return interference;
	
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
		strcpy (string,"robot_"); 
		char id[3];
		itoa(ID_ROBOT, id, 10);  
		strcat(string,id);
		strcat(string,"/odom"); //string = "robot_X/odom" 	
		//strcpy( msg.header.frame_id, string );
		msg.header.frame_id = string;
	}
	
	msg.pose.pose.position.x = xPos[idx]; //send odometry.x
	msg.pose.pose.position.y = yPos[idx]; //send odometry.y
	
	odom_pub.publish(msg);
	ros::spinOnce();
	
   	//ROS_INFO("[POSITION PUBLISHED]: ID = %s, X = %f, Y = %f", msg.header.frame_id.c_str(), xPos[idx], yPos[idx]);
// 	printf("ID_ROBOT = %d\n", ID_ROBOT);
// 	printf("TEAMSIZE = %d\n", TEAMSIZE);	
	check_interference();	
}


int main(int argc, char** argv){	//pass the .graph file to open
  /*
  argc=3
  argv[0]=/.../patrolling_sim/bin/Conscientious_Reactive
  argv[1]=__name:=XXXXXX
  argv[2]=maps/1r-5-map.graph
  argv[3]=ID_ROBOT
  */
  
  
  //More than One robot (ID between 0 and 99)
  if ( atoi(argv[3])>NUM_MAX_ROBOTS || atoi(argv[3])<-1 ){
    ROS_INFO("The Robot's ID must be an integer number between 0 an 99"); //max 100 robots 
    return 0;
  }else{
    ID_ROBOT = atoi(argv[3]); 
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

  ros::init(argc, argv, "c_reactive");
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
 
//   printf("initial position: x = %f, y = %f\n", initial_x, initial_y);
  uint current_vertex = IdentifyVertex(vertex_web, dimension, initial_x, initial_y);
//   printf("initial vertex = %d\n\n",current_vertex);
  
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
    
  
  /* Define Goal */  
  
  if(ID_ROBOT==-1){ 
    strcpy (string,"move_base"); //string = "move_base"  
  }else{ 
    strcpy (string,"robot_"); 
    char id[3];
    itoa(ID_ROBOT, id, 10);  
    strcat(string,id);
    strcat(string,"/move_base"); //string = "robot_X/move_base"  
  }
  
  //printf("string = %s\n",string);
  MoveBaseClient ac(string, true); 
  
  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
     ROS_INFO("Waiting for the move_base action server to come up");
  }  

  //Define Goal:
  move_base_msgs::MoveBaseGoal goal;
  
  //Publicar dados para "results"
  results_pub = nh.advertise<geometry_msgs::PointStamped>("results", 1); //only concerned about the most recent
  results_sub = nh.subscribe("results", 10, resultsCB); //Subscrever "results" vindo dos robots
  
  initialize_node(); //dizer q está vivo
  ros::Rate loop_rate(1); //1 segundo
  
  /* Wait until all nodes are ready.. */
  while(initialize){
	ros::spinOnce();
	loop_rate.sleep();
  }
  
  /* Run Algorithm */
  
  //instantaneous idleness and last visit initialized with zeros:
  double instantaneous_idleness [dimension];
  double last_visit [dimension];
  for(i=0;i<dimension;i++){ 
    instantaneous_idleness[i]= 0.0; 
    last_visit[i]= 0.0; 
    
    if(i==current_vertex){
      last_visit[i]= 0.1; //Avoids getting back at the initial vertex
    }
  }
  
  interference = false;
  ResendGoal = false;
  goal_complete = true;

  
  while(1){
	  
    if(goal_complete){
	    
	    if(next_vertex>-1){
		//Update Idleness Table:
		double now = ros::Time::now().toSec();
			
		for(i=0; i<dimension; i++){
			if (i == next_vertex){
				last_visit[i] = now;	
			}	
			instantaneous_idleness[i]= now - last_visit[i];           
		} 
				
		current_vertex = next_vertex;
			
		//Show Idleness Table:
	/*	for (i=0; i<dimension; i++){
			printf("idleness[%u] = %f\n",i,instantaneous_idleness[i]);      
		} */
	    }
    
	//devolver proximo vertex tendo em conta apenas as idlenesses individuais;
	next_vertex = (int) conscientious_reactive(current_vertex, vertex_web, instantaneous_idleness);
	//printf("Move Robot to Vertex %d (%f,%f)\n", next_vertex, vertex_web[next_vertex].x, vertex_web[next_vertex].y);
	
	//Send the goal to the robot (Global Map)
	geometry_msgs::Quaternion angle_quat = tf::createQuaternionMsgFromYaw(0.0);
	goal.target_pose.header.frame_id = "map"; 
	goal.target_pose.header.stamp = ros::Time::now();    
	goal.target_pose.pose.position.x = vertex_web[next_vertex].x;
	goal.target_pose.pose.position.y = vertex_web[next_vertex].y;  
	goal.target_pose.pose.orientation = angle_quat;
	ROS_INFO("Sending goal - Vertex %d (%f,%f)\n", next_vertex, vertex_web[next_vertex].x, vertex_web[next_vertex].y);
	ac.sendGoal(goal, boost::bind(&goalDoneCallback, _1, _2), boost::bind(&goalActiveCallback), boost::bind(&goalFeedbackCallback, _1));
	//ac.waitForResult();   
	
	goal_complete = false;
    
    }else{
	if (interference){
						
		// Stop the robot..			
 		ROS_INFO("Interference detected!\n");	
		send_interference();

		//get own "odom" positions...
		ros::spinOnce();		
					
		//Waiting until conflict is solved...
		while(interference){
			interference = check_interference();
			if (goal_complete || ResendGoal){
				interference = false;
			}
		}
			// se saiu é pq interference = false			
	}	    
	    
	if(ResendGoal){
		//Send the goal to the robot (Global Map)
		geometry_msgs::Quaternion angle_quat = tf::createQuaternionMsgFromYaw(0.0);	    
		goal.target_pose.header.frame_id = "map"; 
		goal.target_pose.header.stamp = ros::Time::now();    
		goal.target_pose.pose.position.x = vertex_web[next_vertex].x;
		goal.target_pose.pose.position.y = vertex_web[next_vertex].y;  
		goal.target_pose.pose.orientation = angle_quat;	//alpha -> orientação  (queria optimizar este parametro -> através da direcção do vizinho!)
		ROS_INFO("Sending goal - Vertex %d (%f,%f)\n", next_vertex, vertex_web[next_vertex].x, vertex_web[next_vertex].y);
// 		printf("ID_ROBOT = %d\n", ID_ROBOT);
		ac.sendGoal(goal, boost::bind(&goalDoneCallback, _1, _2), boost::bind(&goalActiveCallback), boost::bind(&goalFeedbackCallback, _1));
		ResendGoal = false; //para nao voltar a entrar (envia goal so uma vez)
	}
	
	if(end_simulation){
	      return 0;
	}	
	
    }
  }
  
  return 0; 
}