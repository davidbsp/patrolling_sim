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
* Author: David Portugal (2011-2014), and Luca Iocchi (2014)
*********************************************************************/

#include <sys/stat.h>
#include <sys/types.h>
#include <stdlib.h>
#include <unistd.h>
#include <time.h>
#include <float.h>
#include <fstream>
#include <iostream>
#include <string>

#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
//#include <geometry_msgs/PointStamped.h>	
#include <std_msgs/Int16MultiArray.h>

using namespace std;

#include "getgraph.h"

#define NUM_MAX_ROBOTS 32
#define MAX_COMPLETE_PATROL 100
#define MAX_EXPERIMENT_TIME 86400  // seconds
#define DEAD_ROBOT_TIME 300 // (seconds) time from last goal reached after which a robot is considered dead
#define FOREVER true
// For hystograms
#define RESOLUTION 1.0 // seconds
#define MAXIDLENESS 500.0 // seconds

#include "message_types.h"

using std::cout;
using std::endl;

typedef unsigned int uint;

ros::Subscriber results_sub;
ros::Publisher results_pub;

//Initialization:
bool initialize = true; // Initialization flag
uint cnt=0;  // Count number of robots connected
uint teamsize;
bool init_robots[NUM_MAX_ROBOTS];
double last_goal_reached[NUM_MAX_ROBOTS];

//State Variables:
bool interference = false;
bool goal_reached = false;
int id_robot; // robot sending the message
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


void resultsCB(const std_msgs::Int16MultiArray::ConstPtr& msg) { // msg array: [ID,vertex,intention,interference]

    std::vector<signed short>::const_iterator it = msg->data.begin();    
    
    std::vector<int> vresults;
    
    vresults.clear();
    
    for (int k=0; k<msg->data.size(); k++) {
        vresults.push_back(*it); it++;
    }

    id_robot = vresults[0];
    int msg_type = vresults[1];
        
/*
    int p1 = *it; //data[0]
    ++it;
    int p2 = *it; //data[1]
    ++it;
    int p3 = *it; //data[2]
    ++it;
    int p4 = *it; //data[2]
    ++it;  
*/
    switch(msg_type) {
        case INITIALIZE_MSG_TYPE:
        {
        if (initialize && vresults[2]==1){ 
            if (init_robots[id_robot] == false){ 	//receive init msg: "ID,msg_type,1"
                printf("Robot [ID = %d] is Active!\n", id_robot);
                init_robots[id_robot] = true;
                cnt++;
            } 
            if (cnt==teamsize){
                printf("All Robots GO!\n");
                initialize = false;
                
                //Clock Reset:
                time_zero = ros::Time::now().toSec();
                last_report_time = time_zero; 
                printf("Time zero = %f (s)\n", time_zero);

                std_msgs::Int16MultiArray msg;	// -1,msg_type,0,0,0
                msg.data.clear();
                msg.data.push_back(-1);
                msg.data.push_back(INITIALIZE_MSG_TYPE);
                msg.data.push_back(100);  // Go !!!
                results_pub.publish(msg);
                ros::spinOnce();			
            }
            
        }
        break;
        }
        
        case TARGET_REACHED_MSG_TYPE:
        {
            //goal sent by a robot during the experiment [ID,msg_type,vertex,intention,0]
            if (initialize==false){ 
                goal = vresults[2];
                ROS_INFO("Robot %d reached Goal %d.\n", id_robot, goal); 
                fflush(stdout);
                goal_reached = true;
		ros::spinOnce();
            }
            break;
        }
         
        case INTERFERENCE_MSG_TYPE:
        {
            //interference: [ID,msg_type]
            if (initialize==false){
                ROS_INFO("Robot %d sent interference.\n", id_robot); 
                interference = true;
		ros::spinOnce();
            }
	
            /*else{
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
            }*/
            break;
        }
    }
}

void finish_simulation (){ //-1,msg_type,1,0,0
	std_msgs::Int16MultiArray msg;	
	msg.data.clear();
	msg.data.push_back(-1);
	msg.data.push_back(INITIALIZE_MSG_TYPE);
	msg.data.push_back(999);  // end of the simulation
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

void scenario_name(char* name, const char* graph_file, const char* teamsize_str)
{
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
    strcat(name,teamsize_str);
}

//write_results (avg_idleness, stddev_idleness, number_of_visits, worst_avg_idleness, avg_graph_idl, median_graph_idl, stddev_graph_idl, avg_stddev_graph_idl, min_idleness, gavg, gstddev, max_idleness, interference_cnt, graph_file, algorithm, teamsize_str);
void write_results (double *avg_idleness, double *stddev_idleness, int *number_of_visits, uint complete_patrol, uint dimension, 
                    double worst_avg_idleness, double avg_graph_idl, double median_graph_idl, double stddev_graph_idl, double avg_stddev_graph_idl, 
   double min_idleness, double gavg, double gstddev, double max_idleness, uint interference_cnt, 
                    const char* graph_file, const char* algorithm, const char* teamsize_str, double timevalue, const char *filename){
	FILE *file;
	
    printf("writing to file %s\n",filename);
    // printf("graph file %s\n",graph_file);
        
	file = fopen (filename,"a");
	
	//fprintf(file,"%i\n%i\n%i\n\n",num_nos,largura(),altura());
	fprintf(file, "\nComplete Patrol Cycles:\t%u\n\n", complete_patrol);
	fprintf(file, "Vertex\tAvg Idl\tStdDev Idl\t#Visits\n");

    uint i,tot_visits=0;
    for (i=0; i<dimension; i++){
        fprintf(file, "%u\t%f\t%f\t%d\n", i, avg_idleness[i], stddev_idleness[i], number_of_visits[i] );
        tot_visits += number_of_visits[i];
    }
    float avg_visits = (float)tot_visits/dimension;
    fprintf(file,"\nWorst Avg Graph Idl\t%f\nAvg Avg Graph Idl\t%f\nMedian Avg Graph Idl\t%f\nStdDev Avg Graph Idl\t%f\nAvg StdDevGraph Idl\t%f\nWorst Idl\t%f\n",
    worst_avg_idleness,avg_graph_idl,median_graph_idl,stddev_graph_idl,avg_stddev_graph_idl,max_idleness);
    
    fprintf(file,"\nGlobal Idleness\nAvg\t%.2f\nStddev\t%.2f\nMax\t%.2f\nMin\t%.2f\n",
    gavg,gstddev,max_idleness,min_idleness);

    fprintf(file,"\nInterferences\t%u\nVisits\t%u\nAvg visits\t%.1f\nTime Elapsed\t%f\n",
            interference_cnt,tot_visits,avg_visits,timevalue);
    
    //fprintf(file,"%.2f\t%.2f\t%.2f\t%u\t%u\t%.2f\t%.2f\n",avg_graph_idl,avg_stddev_graph_idl,max_idleness,interference_cnt,tot_visits,avg_visits,timevalue);
    
    fprintf(file,"----------------------------------------------------------------------------------------------------------------------------------------------------------------\n\n\n");    
    
    
	fclose(file); /*done!*/	
}

bool check_dead_robots() {
    double current_time = ros::Time::now().toSec();
    for (int i=0; i<teamsize; i++) {        
      double delta = current_time - last_goal_reached[i];
      // printf("DEBUG dead robot: %d   %.1f - %.1f = %.1f\n",i,current_time,last_goal_reached[i],delta);
      if (delta>DEAD_ROBOT_TIME*0.75) {
        printf("Robot %d: dead robot - delta = %.1f / %.1f \n",i,delta,DEAD_ROBOT_TIME);
        system("play beep.wav");
      }
      if (delta>DEAD_ROBOT_TIME) {
          // printf("Dead robot %d. Time from last goal reached = %.1f\n",i,delta);
          return true;
      }
    }
    return false;
}

int main(int argc, char** argv){	//pass TEAMSIZE GRAPH ALGORITHM
	/*
	argc=3
	argv[0]=/.../patrolling_sim/bin/monitor
	argv[1]=grid
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
	
  string mapname = string(argv[1]);
  string graph_file = "maps/"+mapname+"/"+mapname+".graph";

	printf("Graph: %s\n",graph_file.c_str());
	
	//Check Graph Dimension:
	uint dimension = GetGraphDimension(graph_file.c_str());
	printf("Dimension: %u\n",dimension);
   
    char hostname[80];
    
    int r = gethostname(hostname,80);
    if (r<0)
        strcpy(hostname,"default");
    
    printf("Host name: %s\n",hostname);
   
    
	/* ESTRUTURAS DE DADOS A CALCULAR */
	double last_visit [dimension], current_idleness [dimension], avg_idleness [dimension], stddev_idleness [dimension];
    double total_0 [dimension], total_1 [dimension],  total_2[dimension];
	int number_of_visits [dimension];
	
	double worst_avg_idleness, avg_graph_idl, median_graph_idl, stddev_graph_idl, avg_stddev_graph_idl, previous_avg_graph_idl = DBL_MAX;
  // global measures
  double min_idleness = 0.0, max_idleness = 0.0;
  double gavg, gstddev;
  double gT0=0.0, gT1=0.0, gT2=0.0;

	uint interference_cnt = 0;
	uint complete_patrol = 0;
	uint patrol_cnt = 1;
	
	for (i=0; i<dimension; i++){
		number_of_visits[i] = -1;  // first visit should not be cnted for avg
		current_idleness[i] = 0.0;
		last_visit[i] = 0.0;
	}
	
	for (i=0; i<NUM_MAX_ROBOTS; i++){
		init_robots[i] = false;
        last_goal_reached[i] = 0.0;
	}

	bool dead = false; // check if there is a dead robot
    
    
    // Scenario name (to be used in file and directory names)
    char sname[80];
    scenario_name(sname,graph_file.c_str(),teamsize_str);
    

    // Create directory results if does not exist
    const char* path1 = "results";
    char path2[100],path3[150],path4[200];
    sprintf(path2,"%s/%s",path1,sname);
    sprintf(path3,"%s/%s",path2,algorithm);
    sprintf(path4,"%s/%s",path3,hostname);
    
    struct stat st;
    
    if (stat(path1, &st) != 0)
      mkdir(path1, 0777);
    if (stat(path2, &st) != 0)
      mkdir(path2, 0777);
    if (stat(path3, &st) != 0)
      mkdir(path3, 0777);
    if (stat(path4, &st) != 0)
      mkdir(path4, 0777);

    printf("Path experimental results: %s\n",path4);
    
    // Local time (real clock time)
    time_t rawtime;
    struct tm * timeinfo;
    char strnow[80];
    
    time (&rawtime);
    timeinfo = localtime(&rawtime);
    sprintf(strnow,"%d%02d%02d_%02d%02d%02d",  timeinfo->tm_year+1900,timeinfo->tm_mon+1,timeinfo->tm_mday,timeinfo->tm_hour,timeinfo->tm_min,timeinfo->tm_sec);
    printf("Date-time of the experiment: %s\n",strnow);
    
    // File to log all the idlenesses of an experimental scenario

    char idlfilename[240],resultsfilename[240];
    sprintf(idlfilename,"%s/%s_idleness.csv",path4,strnow);
    sprintf(resultsfilename,"%s/%s_results.csv",path4,strnow);
    
    // Idleness file
    FILE *idlfile;
    idlfile = fopen (idlfilename,"a");
    
    
    
    // Vector fot hystograms
    int hn = (int)(MAXIDLENESS/RESOLUTION)+1;
    int hv[hn]; for (int k=0; k<hn; k++) hv[k]=0;
    int hsum=0;
    
	//Wait for all robots to connect! (Exchange msgs)
	ros::init(argc, argv, "monitor");
	ros::NodeHandle nh;
	
	//Subscrever "results" vindo dos robots
	results_sub = nh.subscribe("results", 100, resultsCB); 	
	
	//Publicar dados para "results"
	results_pub = nh.advertise<std_msgs::Int16MultiArray>("results", 100);
	
  listener = new tf::TransformListener();
    
 	ros::Rate loop_rate(30); //0.033 seconds or 30Hz
	
  nh.setParam("/simulation_runnning", true);
    
  double current_time = ros::Time::now().toSec();
  
  
	while( ros::ok() ){
		
		if (!initialize){	//check if msg is goal or interference -> compute necessary results.
			
			if (interference){
				interference_cnt++;
				interference = false;
			}
			
			if (goal_reached){
				
// 				printf("last_visit [%d] = %f\n", goal, last_visit [goal]);
				current_time = ros::Time::now().toSec();
				printf("Robot %d reached goal %d (current time: %f)\n", id_robot, goal, current_time);
                
				double last_visit_temp = current_time - time_zero; ; //guarda o valor corrente
				number_of_visits [goal] ++;
				
        last_goal_reached[id_robot] = current_time;
                
 				printf("  number_of_visits [%d] = %d\n", goal, number_of_visits [goal]);

                if (number_of_visits [goal] == 0) {
                    avg_idleness [goal] = 0.0; stddev_idleness[goal] = 0.0;
                    total_0 [goal] = 0.0; total_1 [goal] = 0.0;  total_2 [goal] = 0.0;
                }
                else { // if (number_of_visits [goal] > 0) {

                    current_idleness [goal] = last_visit_temp - last_visit [goal];
                    printf("  current_idleness [%d] = %f\n", goal, current_idleness [goal]);
                
                    if (current_idleness [goal] > max_idleness)
                        max_idleness=current_idleness [goal];
                    if (current_idleness [goal] < min_idleness || min_idleness<0.1)
                        min_idleness=current_idleness [goal];
                    // global stats
                    gT0++; gT1 += current_idleness[goal]; gT2 += current_idleness[goal]*current_idleness[goal];

                
                    fprintf(idlfile,"%.1f;%d;%d;%.1f;%d\n",current_time,id_robot,goal,current_idleness[goal],interference_cnt);
                    fflush(idlfile);

                    // for hystograms
                    int b = (int)(current_idleness[goal]/RESOLUTION);
                    hv[b]++; hsum++;
            
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
                printf("  max_idleness = %f\n", max_idleness);
                printf("  interferences = %d\n", interference_cnt);
                
				last_visit [goal] = last_visit_temp;
// 				printf("last_visit [%d] (UPDATED) = %f\n\n", goal, last_visit [goal]);
				
				complete_patrol = calculate_patrol_cycle ( number_of_visits, dimension );                
                printf("  complete patrol cycles = %d\n\n", complete_patrol);
                
                
				goal_reached = false;
			}
			
			// check time
			double report_time = ros::Time::now().toSec();
            //printf("report time=%.1f\n",report_time);
			
			if ((patrol_cnt == complete_patrol) || (report_time - last_report_time>MAX_EXPERIMENT_TIME)){ 
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
        // global stats
        gavg = gT1/gT0;
        gstddev = 1.0/gT0 * sqrt(gT0*gT2-gT1*gT1);

        printf("Node idleness\n");
				printf("   worst_avg_idleness (graph) = %f\n", worst_avg_idleness);
				printf("   avg_idleness (graph) = %f\n", avg_graph_idl);
				median_graph_idl = Median (avg_idleness, dimension);
        printf("   median_idleness (graph) = %f\n", median_graph_idl);
        printf("   stddev_idleness (graph) = %f\n", stddev_graph_idl);

        printf("Global idleness\n");
        printf("   min = %.1f\n", min_idleness);
        printf("   avg = %.1f\n", gavg);
        printf("   stddev = %.1f\n", gstddev);
        printf("   max = %.1f\n", max_idleness);

				printf("Interferences = %d\n\n", interference_cnt);
				

        

				patrol_cnt++;
				last_report_time = report_time;
                
				double tolerance = 0.025 * avg_graph_idl;	//2.5% tolerance
				printf ("diff avg_idleness = %f\n",fabs(previous_avg_graph_idl - avg_graph_idl));
				printf ("tolerance = %f\n",tolerance);
				
				//Write to File:
				write_results (avg_idleness, stddev_idleness, number_of_visits, complete_patrol, dimension, 
                               worst_avg_idleness, avg_graph_idl, median_graph_idl, stddev_graph_idl, avg_stddev_graph_idl, min_idleness, gavg, gstddev, max_idleness,
                   interference_cnt, graph_file.c_str(), algorithm, teamsize_str,ros::Time::now().toSec()-time_zero,resultsfilename);
                
                // if ((complete_patrol>=MAX_COMPLETE_PATROL) && fabs(previous_avg_graph_idl - avg_graph_idl) <= tolerance)) {
                //    ... simulation is over... 
                // }
			}
			
			// Check if simulation must be terminated
			dead = check_dead_robots();
                
            bool simrun = true;
            std::string psimrun;
            if (nh.getParam("/simulation_runnning", psimrun))
                if (psimrun=="false")
                    simrun = false;
            
            if ( (dead) || (!simrun) ) {
                printf ("Simulation is Over\n");                    
                nh.setParam("/simulation_runnning", false);
                finish_simulation ();
                ros::spinOnce();
                break;
            }


        }	
		
		ros::spinOnce();
		loop_rate.sleep();		
	}
	
	fclose(idlfile);
    
    
    // Hystogram files
    char hfilename[240],chfilename[240];
    sprintf(hfilename, "%s/%s.hist", path4,strnow);
    sprintf(chfilename,"%s/%s.chist",path4,strnow);

    cout << "Histogram output files: " << hfilename << endl;
    std::ofstream of1; of1.open(hfilename);
    std::ofstream of2; of2.open(chfilename);
    double c=0;
    for (int k=0; k<hn; k++) {
        of1 << k*RESOLUTION << " " << (double)hv[k]/hsum << endl;
        c += (double)hv[k]/hsum;
        of2 << k*RESOLUTION << " " << c << endl;
    }
    of1.close();   of2.close();
    
    std::string algparams;
    ros::param::get("/algorithm_params", algparams);

    double goal_reached_wait;
    if (! ros::param::get("/goal_reached_wait", goal_reached_wait))
        goal_reached_wait = 0.0;


    char infofilename[240];
    sprintf(infofilename,"%s/%s_info.csv",path4,strnow);

    FILE *infofile;
    infofile = fopen (infofilename,"w");
    fprintf(infofile,"%s;%s;%.1f;%s;%s;%s;%s;%.1f;%d;%s;%.1f;%.1f;%.1f;%.1f\n",
            mapname.c_str(),teamsize_str,goal_reached_wait,algorithm,
            algparams.c_str(),hostname,
            strnow,current_time,interference_cnt,(dead?"FAIL":"TIMEOUT"),
            min_idleness, gavg, gstddev, max_idleness
    );

    fclose(infofile);
    
	printf("Monitor closed.\n");
	usleep(1e9);
	
}
