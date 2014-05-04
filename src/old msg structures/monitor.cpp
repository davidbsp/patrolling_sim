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
#include <tf/transform_listener.h>
#include <geometry_msgs/PointStamped.h>	
#include <float.h>

#include "getgraph.h"

#define NUM_MAX_ROBOTS 32

typedef unsigned int uint;

ros::Subscriber results_sub;
ros::Publisher results_pub;

//Initialization:
bool initialize = true;
uint count=0;
uint teamsize;
bool init_robots[NUM_MAX_ROBOTS];

//State Variables:
bool interference = false;
bool goal_reached = false;
int goal;
double time_zero, last_report_time;

tf::TransformListener *listener;

void getRobotPose(int robotid, float &x, float &y, float &theta) {
    
    std::string robotname = "robot_"+robotid;
    tf::StampedTransform transform;

    try {
        listener->waitForTransform("/map", "/" + robotname + "/base_link", ros::Time(0), ros::Duration(3));
        listener->lookupTransform("/map", "/" + robotname + "/base_link", ros::Time(0), transform);
    }
    catch(tf::TransformException ex) {
        ROS_ERROR("%s", ex.what());
    }

    x = transform.getOrigin().x();
    y = transform.getOrigin().y();
    theta = tf::getYaw(transform.getRotation());
}


void resultsCB(const geometry_msgs::PointStamped::ConstPtr& msg) {	//Entra aqui sempre que recebe uma msg
/*
Header header
    uint32 seq		//msg seq
    time stamp		//time
    string frame_id	//robot ID
Point point
    float64 x		//vertice (X,0,0)
    float64 y		//interference (0,Y,0)
    float64 z		//initialization (0,0,Z=ID)
*/
// 	printf("Monitor: Results callback. frame_id = %s (%f,%f,%f)\n",msg->header.frame_id.c_str(),msg->point.x,msg->point.y,msg->point.z);
	int id_robot_int;
	
	if (initialize){
			
		double id_robot_dbl = msg->point.z;
		id_robot_int = (int) id_robot_dbl;
		
		//printf("Dados:\nseq = %d\nframe_id = %s\nx = %f\ny = %f\n",msg->header.seq, msg->header.frame_id.c_str(),msg->point.x,msg->point.y);
		
		if (id_robot_int == -1){id_robot_int = 0;}
		if (init_robots[id_robot_int] == false){
			printf("Robot [ID = %d] is Active!\n",id_robot_int);
			init_robots[id_robot_int] = true;
			count++;
		}
			
		if (count==teamsize){
			printf("All Robots GO!\n");
			initialize = false;
			
			//Clock Reset:
			time_zero = ros::Time::now().toSec();
            last_report_time = time_zero; 
			printf("Time zero = %f (s)\n", time_zero);

			geometry_msgs::PointStamped msg;	
			msg.header.frame_id = "monitor";
			msg.point.x = 0.0;
			msg.point.y = 0.0; 
			msg.point.z = 0.0;
			results_pub.publish(msg);
			ros::spinOnce();
		}
		
	}else{
		//Interferencia ou Goal
		double vertex = msg->point.x;
		double interf = msg->point.y;
		double init = msg->point.z;
		
		if (msg->header.frame_id != "monitor"){
			
// 			printf("Interference or Goal! frame_id = %s (%f,%f)\n",msg->header.frame_id.c_str(),msg->point.x,msg->point.y);
			id_robot_int = atoi( msg->header.frame_id.c_str() ); 
			
			if (interf == 0.0 && init == 0.0){
				goal = (int) vertex;
				printf("Robot %d reached Goal %d.\n", id_robot_int, goal);
				goal_reached = true;
			}
			if (interf == 1.0 ){
// 				printf("Received Interference from Robot %d.\n", id_robot_int);
				interference = true;
			}
			
		}
	}
}

void finish_simulation (){
	geometry_msgs::PointStamped msg;	
	msg.header.frame_id = "monitor";
	msg.point.x = 1.0;
	msg.point.y = 1.0; 
	msg.point.z = 1.0;
	results_pub.publish(msg);
	ros::spinOnce();
}

// return the median value in a vector of size "dimension" floats pointed to by a
double Median( double *a, uint dimension )
{
   uint table_size = dimension/2;   
   if(dimension % 2 != 0){ //odd
	 table_size++; 
   }   
   if (table_size==0) {table_size = 1;}
   
   double left[table_size], right[table_size], median, *p;
   unsigned char nLeft, nRight;

   // pick first value as median candidate
   p = a;
   median = *p++;
   nLeft = nRight = 1;

   for(;;)
   {
       // get next value
       double val = *p++;

       // if value is smaller than median, append to left heap
       if( val < median )
       {
           // move biggest value to the heap top
           unsigned char child = nLeft++, parent = (child - 1) / 2;
           while( parent && val > left[parent] )
           {
               left[child] = left[parent];
               child = parent;
               parent = (parent - 1) / 2;
           }
           left[child] = val;

           // if left heap is full
           if( nLeft == table_size )
           {
               // for each remaining value
               for( unsigned char nVal = dimension - (p - a); nVal; --nVal )
               {
                   // get next value
                   val = *p++;

                   // if value is to be inserted in the left heap
                   if( val < median )
                   {
                       child = left[2] > left[1] ? 2 : 1;
                       if( val >= left[child] )
                           median = val;
                       else
                       {
                           median = left[child];
                           parent = child;
                           child = parent*2 + 1;
                           while( child < table_size )
                           {
                               if( child < table_size-1 && left[child+1] > left[child] )
                                   ++child;
                               if( val >= left[child] )
                                   break;
                               left[parent] = left[child];
                               parent = child;
                               child = parent*2 + 1;
                           }
                           left[parent] = val;
                       }
                   }
               }
               return median;
           }
       }

       // else append to right heap
       else
       {
           // move smallest value to the heap top
           unsigned char child = nRight++, parent = (child - 1) / 2;
           while( parent && val < right[parent] )
           {
               right[child] = right[parent];
               child = parent;
               parent = (parent - 1) / 2;
           }
           right[child] = val;

           // if right heap is full
           if( nRight == 14 )
           {
               // for each remaining value
               for( unsigned char nVal = dimension - (p - a); nVal; --nVal )
               {
                   // get next value
                   val = *p++;

                   // if value is to be inserted in the right heap
                   if( val > median )
                   {
                       child = right[2] < right[1] ? 2 : 1;
                       if( val <= right[child] )
                           median = val;
                       else
                       {
                           median = right[child];
                           parent = child;
                           child = parent*2 + 1;
                           while( child < table_size )
                           {
                               if( child < 13 && right[child+1] < right[child] )
                                   ++child;
                               if( val <= right[child] )
                                   break;
                               right[parent] = right[child];
                               parent = child;
                               child = parent*2 + 1;
                           }
                           right[parent] = val;
                       }
                   }
               }
               return median;
           }
       }
   }
}


uint calculate_patrol_cycle ( int *nr_visits, uint dimension ){
	int result = INT_MAX;
	int imin=0;
	for (int i=0; i<(int)dimension; i++){
		if (nr_visits[i] < result){
			result = nr_visits[i]; imin=i;
		}
	}
	//printf("  --- complete patrol: visits of %d : %d\n",imin,result);
	return result;	
}

//write_results (avg_idleness, stddev_idleness, number_of_visits, worst_avg_idleness, avg_graph_idl, median_graph_idl, stddev_graph_idl, avg_stddev_graph_idl, worst_idleness, interference_count, graph_file, algorithm, teamsize_str);
void write_results (double *avg_idleness, double *stddev_idleness, int *number_of_visits, uint complete_patrol, uint dimension, 
                    double worst_avg_idleness, double avg_graph_idl, double median_graph_idl, double stddev_graph_idl, double avg_stddev_graph_idl, double worst_idleness, uint interference_count, 
                    char* graph_file, char* algorithm, char* teamsize_str, double timevalue){
	FILE *file;
	
    printf("writing to file \n");
    printf("graph file %s\n",graph_file);
    
    char file_path[120];
	char name[120];
    
	uint i, start_char=0, end_char = strlen(graph_file)-1;
	
	for (i=0; i<strlen(graph_file); i++){
		if(graph_file[i]=='/' && i < strlen(graph_file)-1){
			start_char = i+1;
		}
		
		if(graph_file[i]=='.' && i>0){
			end_char = i-1;
			break;
		}
	}
	
	for (i=start_char; i<=end_char; i++){
		name [i-start_char] = graph_file [i];
		if (i==end_char){
			name[i-start_char+1] = '\0';
		}
	}
	
	strcat(name,"_");
	strcat(name,algorithm);
	strcat(name,"_");
	strcat(name,teamsize_str);
	strcat(name,".xls");
		
	
	strcpy(file_path,"results/");
	strcat(file_path,name);
	
	printf("FILE: %s\n", file_path);	
	file = fopen (file_path,"a");
	
	//fprintf(file,"%i\n%i\n%i\n\n",num_nos,largura(),altura());
	fprintf(file, "\nComplete Patrol Cycles:\t%u\n\n", complete_patrol);
	fprintf(file, "Vertex\tAvg Idl\tStdDev Idl\t#Visits\n");
	
	for (i=0; i<dimension; i++){
		fprintf(file, "%u\t%f\t%f\t%d\n", i, avg_idleness[i], stddev_idleness[i], number_of_visits[i] );
	}
	
	fprintf(file,"\nWorst Avg Graph Idl\t%f\nAvg Avg Graph Idl\t%f\nMedian Avg Graph Idl\t%f\nStdDev Avg Graph Idl\t%f\nAvg StdDevGraph Idl\t%f\nWorst Idl\t%f\nInterferences\t%u\nTime Elapsed\t%f\n",
            worst_avg_idleness,avg_graph_idl,median_graph_idl,stddev_graph_idl,avg_stddev_graph_idl,worst_idleness,interference_count,timevalue);
    
    fprintf(file,"----------------------------------------------------------------------------------------------------------------------------------------------------------------\n\n\n");
    
    
    
	fclose(file); /*done!*/	
}


int main(int argc, char** argv){	//pass TEAMSIZE GRAPH ALGORITHM
	/*
	argc=3
	argv[0]=/.../patrolling_sim/bin/monitor
	argv[1]=maps/1r-5-map.graph
	argv[2]=ALGORITHM = {MSP,Cyc,CC,CR,HCR}
	argv[3]=TEAMSIZE
	*/
	
	//ex: "rosrun patrolling_sim monitor maps/example/example.graph MSP 2"
  
// 	uint teamsize;
	char teamsize_str[3];
	teamsize = atoi(argv[3]);
	
	if ( teamsize >= NUM_MAX_ROBOTS || teamsize <1 ){
		ROS_INFO("The Teamsize must be an integer number between 1 and %d", NUM_MAX_ROBOTS);
		return 0;
	}else{
		strcpy (teamsize_str, argv[3]); 
// 		printf("teamsize: %s\n", teamsize_str);
// 		printf("teamsize: %u\n", teamsize);
	}
	
	uint i = strlen( argv[2] );
	char algorithm[i];
	strcpy (algorithm,argv[2]);
	printf("Algorithm: %s\n",algorithm);
	
	i = strlen( argv[1] );
	char graph_file[i];
	strcpy (graph_file,argv[1]);
	printf("Graph: %s\n",graph_file);
	
	//Check Graph Dimension:
	uint dimension = GetGraphDimension(graph_file);
	printf("Dimension: %u\n",dimension);
   
	/* ESTRUTURAS DE DADOS A CALCULAR */
	double last_visit [dimension], current_idleness [dimension], avg_idleness [dimension], stddev_idleness [dimension];
    double total_0 [dimension], total_1 [dimension],  total_2[dimension];
	int number_of_visits [dimension];
	
	double worst_avg_idleness, avg_graph_idl, median_graph_idl, stddev_graph_idl, avg_stddev_graph_idl, previous_avg_graph_idl = DBL_MAX;
    double worst_idleness = 0.0;
	uint interference_count = 0;
	uint complete_patrol = 0;
	uint patrol_count = 1;
	
	for (i=0; i<dimension; i++){
		number_of_visits[i] = -1;  // first visit should not be counted for avg
		current_idleness[i] = 0.0;
		last_visit[i] = 0.0;
	}	
	
	for (i=0; i<NUM_MAX_ROBOTS; i++){
		init_robots[i] = false;
	}
	
	//Wait for all robots to connect! (Exchange msgs)
	ros::init(argc, argv, "monitor");
	ros::NodeHandle nh;
	
	//Subscrever "results" vindo dos robots
	results_sub = nh.subscribe("results", 10, resultsCB); 	
	
	//Publicar dados para "results"
	results_pub = nh.advertise<geometry_msgs::PointStamped>("results", 1); //only concerned about the most recent
	
    listener = new tf::TransformListener();
    
 	ros::Rate loop_rate(10); //0.1 segundos 
	 
	while( ros::ok() ){
		
		if (!initialize){	//check if msg is goal or interference -> compute necessary results.
			
			if (interference){
				interference_count++;
				interference = false;
			}
			
			if (goal_reached){
				
// 				printf("last_visit [%d] = %f\n", goal, last_visit [goal]);
				double current_time = ros::Time::now().toSec();
                printf("Reached goal %d (current time: %f)\n", goal, current_time);
                
				double last_visit_temp = current_time - time_zero; ; //guarda o valor corrente
				number_of_visits [goal] ++;
				
 				printf("  number_of_visits [%d] = %d\n", goal, number_of_visits [goal]);
				
				current_idleness [goal] = last_visit_temp - last_visit [goal];
 				printf("  current_idleness [%d] = %f\n", goal, current_idleness [goal]);

                if (current_idleness [goal] > worst_idleness)
                    worst_idleness=current_idleness [goal];
                
                if (number_of_visits [goal] == 0) {
                    avg_idleness [goal] = 0.0; stddev_idleness[goal] = 0.0;
                    total_0 [goal] = 0.0; total_1 [goal] = 0.0;  total_2 [goal] = 0.0;
                }
                else { // if (number_of_visits [goal] == 1) {
					// avg_idleness [goal] = current_idleness [goal];
                    total_0 [goal] += 1.0; total_1 [goal] += current_idleness [goal];  total_2 [goal] += current_idleness [goal]*current_idleness [goal];
                    avg_idleness [goal] = total_1[goal]/total_0[goal]; 
                    stddev_idleness[goal] = 1.0/total_0[goal] * sqrt(total_0[goal]*total_2[goal]-total_1[goal]*total_1[goal]); 
				}
				/*
				else {	// n. of visits >= 2
					avg_idleness [goal] = ( avg_idleness [goal] * (double) (number_of_visits [goal] - 1)  + current_idleness [goal] ) / ( (double) number_of_visits [goal] );
                    if (number_of_visits [goal] == 2)
                        var_idleness[goal] = 0.0;
                    else
                        var_idleness[goal] = 
				}*/
				
 				printf("  avg_idleness [%d] = %f, stddev_idleness [%d] = %f \n", goal, avg_idleness [goal], goal, stddev_idleness[goal]);
                printf("  worst_idleness = %f\n", worst_idleness);
                printf("  interferences = %d\n", interference_count);
                
				last_visit [goal] = last_visit_temp;
// 				printf("last_visit [%d] (UPDATED) = %f\n\n", goal, last_visit [goal]);
				
				complete_patrol = calculate_patrol_cycle ( number_of_visits, dimension );                
                printf("  complete patrol cycles = %d\n\n", complete_patrol);
                
                
				goal_reached = false;
			}
			
			// check time
			double report_time = ros::Time::now().toSec();
            //printf("report time=%.1f\n",report_time);
			
			if ((patrol_count == complete_patrol) || (report_time - last_report_time>3600)){ 
                    //write results every time a patrolling cycle is finished.
                    //or after some time
				previous_avg_graph_idl = avg_graph_idl; //save previous avg idleness graph value

				printf("Patrol completed [%d]. Write to File!\n",complete_patrol);
				worst_avg_idleness = 0.0;
				avg_graph_idl = 0.0;
				stddev_graph_idl = 0.0;
                avg_stddev_graph_idl = 0.0;
                
                double T0=0.0,T1=0.0,T2=0.0,S1=0.0;
				for (i=0; i<dimension; i++){	//escrever avg_idle[vertex] e #visits
					T0++; T1+=avg_idleness[i]; T2+=avg_idleness[i]*avg_idleness[i];
                    S1+=stddev_idleness[i];
					if ( avg_idleness[i] > worst_avg_idleness ){
						worst_avg_idleness = avg_idleness[i];
					}
                }
                
				avg_graph_idl = T1/T0;
                stddev_graph_idl = 1.0/T0 * sqrt(T0*T2-T1*T1);
                avg_stddev_graph_idl = S1/T0;

				printf("worst_avg_idleness (graph) = %f\n", worst_avg_idleness);
				printf("avg_idleness (graph) = %f\n", avg_graph_idl);
				median_graph_idl = Median (avg_idleness, dimension);
                printf("median_idleness (graph) = %f\n", median_graph_idl);
                printf("stddev_idleness (graph) = %f\n", stddev_graph_idl);
                printf("worst_idleness (nodes) = %f\n", worst_idleness);
				printf("interferences = %d\n\n", interference_count);
				
				patrol_count++;
				last_report_time = report_time;
                
				double tolerance = 0.025 * avg_graph_idl;	//2.5% tolerance
				printf ("tolerance = %f\n",tolerance);
				
				//Write to File:
				write_results (avg_idleness, stddev_idleness, number_of_visits, complete_patrol, dimension, 
                               worst_avg_idleness, avg_graph_idl, median_graph_idl, stddev_graph_idl, avg_stddev_graph_idl, worst_idleness, 
                   interference_count, graph_file, algorithm, teamsize_str,ros::Time::now().toSec()-time_zero);
				
				if ((complete_patrol>=10) && abs (previous_avg_graph_idl - avg_graph_idl) <= tolerance) {
					printf ("Simulation is Over\n");
					finish_simulation ();
					return 0;
				}
				
			}
		}	
		
		ros::spinOnce();
		loop_rate.sleep();
	}
	
	printf("Monitor closed.\n");
	usleep(100e6);
	
}