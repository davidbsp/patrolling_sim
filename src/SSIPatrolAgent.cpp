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
* Author: Luca Iocchi (2014-2016)
*********************************************************************/

#include "SSIPatrolAgent.h"

using namespace std;

SSIPatrolAgent::SSIPatrolAgent() : cf(CONFIG_FILENAME)
{
    pthread_mutex_init(&lock, NULL);
}
    

void SSIPatrolAgent::onGoalComplete()
{
    printf("DTAP onGoalComplete!!!\n");
    if (first_vertex){
        //printf("computing next vertex FOR THE FIRST TIME:\n current_vertex = %d, next_vertex=%d, next_next_vertex=%d",current_vertex, next_vertex,next_next_vertex);
        next_vertex = compute_next_vertex(current_vertex);
        //printf("DONE: current_vertex = %d, next_vertex=%d, next_next_vertex=%d\n",current_vertex, next_vertex,next_next_vertex);		
        first_vertex = false;
    } else {
        //printf("updating next vertex :\n current_vertex = %d, next_vertex=%d, next_next_vertex=%d\n",current_vertex, next_vertex,next_next_vertex);
        
        //Update Idleness Table:
        update_global_idleness();
        //update current vertex
        current_vertex = next_vertex;
        //update next vertex based on previous decision
        next_vertex = next_next_vertex;
        //update global idleness of next vertex to avoid conflicts
        
        if (next_vertex>=0 && next_vertex< dimension){
            pthread_mutex_lock(&lock);
            global_instantaneous_idleness[next_vertex] = 0.0;   
            pthread_mutex_unlock(&lock);
        }
        //printf("DONE: current_vertex = %d, next_vertex=%d, next_next_vertex=%d\n",current_vertex, next_vertex,next_next_vertex);		
   }

    /** SEND GOAL (REACHED) AND INTENTION **/
    send_goal_reached(); // Send TARGET to monitor
    send_results();  // Algorithm specific function
    
    //Send the goal to the robot (Global Map)
    ROS_INFO("Sending goal - Vertex %d (%f,%f)\n", next_vertex, vertex_web[next_vertex].x, vertex_web[next_vertex].y);
    //sendGoal(vertex_web[next_vertex].x, vertex_web[next_vertex].y);  
    sendGoal(next_vertex);  // send to move_base

    goal_complete = false; 

    //compute next next vertex
    //printf("computing next_next_vertex :\n current_vertex = %d, next_vertex=%d, next_next_vertex=%d\n",current_vertex, next_vertex,next_next_vertex);
    
    next_next_vertex = compute_next_vertex(next_vertex); 
	   
    printf("<<< DONE Computed next vertices: current_vertex = %d, next_vertex=%d, next_next_vertex=%d >>>\n",current_vertex, next_vertex,next_next_vertex);		

}


bool* SSIPatrolAgent::create_selected_vertices(){
	bool* sv = new bool[dimension];
	for(size_t i=0; i<dimension; i++) {
	        selected_vertices[i] = false;
    	}	
	return sv;
}

void SSIPatrolAgent::reset_selected_vertices(bool* sv){
	for(size_t i=0; i<dimension; i++) {
		sv[i] = false;
    }
    if (current_vertex >= 0 && current_vertex < dimension){
		selected_vertices[current_vertex] = true; //do not consider next vertex as possible goal 
    } 	
}

void SSIPatrolAgent::select_faraway_vertices(bool* sv, int cv){
	for(size_t i=0; i<dimension; i++) {
		sv[i] = true;
    }
	uint num_neighs = vertex_web[cv].num_neigh;
    for (size_t i=0; i<num_neighs; i++){
      size_t neighbor = vertex_web[cv].id_neigh[i];
	  sv[neighbor] = false; 	
	}
    if (current_vertex >= 0 && current_vertex < dimension){
		selected_vertices[current_vertex] = true; //do not consider next vertex as possible goal 
    } 	
	/* print farway vertices
	printf("FARAWAY \n [ ");
	for(size_t i=0; i<dimension; i++) {
		printf("%d ",sv[i]);
    }
	printf("]\n"); */
}

bool SSIPatrolAgent::all_selected(bool* sv){
	for(size_t i=0; i<dimension; i++) {
	    if (!sv[i]){ 
			return false;		
		}	
    }
	return true;
}


void SSIPatrolAgent::init(int argc, char** argv) {
        
    PatrolAgent::init(argc,argv);

    //initialize structures
    next_vertex = -1; 
    next_next_vertex = -1;

    taskRequests = new int[dimension];  
    tasks = new bool[dimension];    
    bids = new bid_tuple[dimension]; 
    global_instantaneous_idleness = new double[dimension];
    selected_vertices = new bool[dimension];
    bid_tuple noBid = {BIG_NUMBER,-1}; 
    for(size_t i=0; i<dimension; i++) {
        taskRequests[i] = 0;
        tasks[i] = false;
        selected_vertices[i] = false;
        bids[i] = noBid;
        global_instantaneous_idleness[i]=dimension*2;  // start with a high value (not too high) 
    }
    nactivetasks=0;

    last_update_idl = ros::Time::now().toSec();

    first_vertex = true;	

    //initialize parameters
    timeout = cf.getDParam("timeout");
    theta_idl = cf.getDParam("theta_idleness");
    theta_cost = cf.getDParam("theta_navigation");
    theta_hop = cf.getDParam("theta_hop");	
    threshold = cf.getDParam("threshold");			
    hist = cf.getDParam("hist");

    std::stringstream paramss;
    paramss << timeout << "," << theta_idl << "," << theta_cost << "," << theta_hop << "," << threshold << "," << hist;

    ros::param::set("/algorithm_params",paramss.str());

}

double SSIPatrolAgent::compute_cost(int vertex)
{
    uint elem_s_path;
    int *shortest_path = new int[dimension]; 
    int id_neigh;
    
    dijkstra( current_vertex, vertex, shortest_path, elem_s_path, vertex_web, dimension); //structure with normal costs
    double distance = 0;
    
    for(uint j=0; j<elem_s_path; j++){
//        printf("path[%u] = %d\n",j,shortest_path[j]);
        
        if (j<elem_s_path-1){
            id_neigh = is_neigh(shortest_path[j], shortest_path[j+1], vertex_web, dimension);
            distance += vertex_web[shortest_path[j]].cost[id_neigh];
        }       
    }
    
    return distance;
} 
        

//TODO this should give the geometric distance from robot position to next vertex, and not path cost from current_vertex to next_vertex
/*double SSIPatrolAgent::compute_distance(int vertex)
{

    uint elem_s_path;
    int *shortest_path = new int[dimension]; 
    int id_neigh;
    
    dijkstra( current_vertex, vertex, shortest_path, elem_s_path, vertex_web, dimension); //structure with normal costs
    double distance = 0;
    
    for(uint j=0; j<elem_s_path; j++){
//        printf("path[%u] = %d\n",j,shortest_path[j]);
        
        if (j<elem_s_path-1){
            id_neigh = is_neigh(shortest_path[j], shortest_path[j+1], vertex_web, dimension);
            distance += vertex_web[shortest_path[j]].cost[id_neigh];
        }       
    }
    
    return distance;
}        
*/

double SSIPatrolAgent::compute_cost(int cv, int nv)
{
    uint elem_s_path;
    int *shortest_path = new int[dimension]; 
    int id_neigh;
    
    dijkstra( cv, nv, shortest_path, elem_s_path, vertex_web, dimension); //structure with normal costs
    double distance = 0;
    
    for(uint j=0; j<elem_s_path; j++){
//        printf("path[%u] = %d\n",j,shortest_path[j]);
        
        if (j<elem_s_path-1){
            id_neigh = is_neigh(shortest_path[j], shortest_path[j+1], vertex_web, dimension);
            distance += vertex_web[shortest_path[j]].cost[id_neigh];
        }       
    }
    
    return distance;
}        


size_t SSIPatrolAgent::compute_hops(int cv, int nv)
{
    uint elem_s_path;
    int *shortest_path = new int[dimension]; 
    int id_neigh;
    
    dijkstra( cv, nv, shortest_path, elem_s_path, vertex_web, dimension); //structure with normal costs
    
#if 1
    return elem_s_path-1;
#else    
    size_t hops = 0;
    
    for(uint j=0; j<elem_s_path; j++){
//        printf("path[%u] = %d\n",j,shortest_path[j]);
        
        if (j<elem_s_path-1){
            id_neigh = is_neigh(shortest_path[j], shortest_path[j+1], vertex_web, dimension);
			hops++;
        }       
    }
    return hops;
#endif
}        


double SSIPatrolAgent::utility(int cv,int nv) {
    double idl = global_instantaneous_idleness[nv];
    
    size_t hops = compute_hops(cv,nv);
    double U = theta_idl * idl + theta_hop * hops * dimension;

    // double cost = compute_cost(cv,nv); ????  1 hop = 5 m
    // double U = theta_idl * idl + theta_navigation * cost;
#if DEBUG_PRINT
    if (U>-1000)
        printf("  HOPSUtil:: node: %d --> U[%d] ( %.1f, %zu ) = %.1f\n",cv,nv,idl,hops,U);
#endif
    return U;
}


void SSIPatrolAgent::update_global_idleness() 
{   
    double now = ros::Time::now().toSec();
    
    pthread_mutex_lock(&lock);
    for(size_t i=0; i<dimension; i++) {
        global_instantaneous_idleness[i] += (now-last_update_idl);  // update value    
    }
    
    if (current_vertex>=0 && current_vertex<dimension){
        global_instantaneous_idleness[current_vertex] = 0.0;
    }
    pthread_mutex_unlock(&lock);

    last_update_idl = now;
}

int SSIPatrolAgent::return_next_vertex(int cv,bool* sv){
    double maxUtility = -1e9;
    int i_maxUtility = 0;

    for(size_t i=0; i<dimension; i++){
        
        double U = utility(cv,i);
        if (U > maxUtility && !sv[i]){
            maxUtility = U;
            i_maxUtility = i;
        }
        
    }
    
    int nv = i_maxUtility; // vertex_web[current_vertex].id_neigh[i_maxUtility];
    sv[nv] = true; //this vertex was considered as a next vertex
    printf("DTASSI: returned vertex = %d (U = %.2f)\n",nv,maxUtility);
    return nv;
}

int SSIPatrolAgent::select_next_vertex(int cv,bool* sv){
    
    double maxUtility = -1e9;
    int i_maxUtility = 0;


    //check if there are vertices not selected, if not reset all to false	
    if (all_selected(selected_vertices)) {
		reset_selected_vertices(sv);
    }
    for(size_t i=0; i<dimension; i++){
        
        double U = utility(cv,i);
//		printf("vertex %d, marked %d",i,sv[i]);
        if (U > maxUtility && !sv[i]){
            maxUtility = U;
            i_maxUtility = i;
        }
    }
    
    int nv = i_maxUtility; // vertex_web[current_vertex].id_neigh[i_maxUtility];
    sv[nv] = true; //this vertex was considered as a next vertex
    // printf("DTASSI: selected possible vertex = %d (U = %.2f)\n",nv,maxUtility);

    return nv;
}

double SSIPatrolAgent::compute_bid(int nv){

	printf("## computing bid for vertex %d \n",nv);
/*	printf("current tasks = ");
	for (size_t i = 0; i<dimension;i++){
		printf(" %d, ",tasks[i]);	
    }
    printf("] \n"); */

	if (nv==next_vertex || nv==next_next_vertex){
//		printf("already going to %d sending 0 (current target: %d, current next target: %d)",nv,next_vertex,next_next_vertex);
		return 0.;
	}

	//local copy of task list for computing the bid, put as selected the ones I do not want to consider 
	bool* my_tasks = new bool[dimension];
	//printf("current tasks [");
	for (size_t i = 0; i<dimension; i++){
	//	printf(" %d, ",tasks[i]);
		my_tasks[i] = !tasks[i];
	} 
	//printf(" ] \n");*/

	//add nv to my_tasks (regardless of whether this was my responsibility already)
	my_tasks[nv] = false;

/*	printf("my tasks [");
	for (size_t i = 0; i<dimension; i++){
		printf(" %d, ",my_tasks[i]);
	} 
	printf(" ] \n");
*/
	//accumulator for total path cost
	double path_cost = 0.;

	//put as first location the current target if any
	int ci = -1;
	if (next_vertex != -1){
		ci = next_vertex;
		path_cost = compute_cost(ci);//this should give the geometric distance from robot position to destination 
		my_tasks[ci] = true; //remove this task from the list
		//printf("[Target Set] Pathcost from %d to %d : %.2f \n",current_vertex,ci,path_cost);
	} else { 
		ci = return_next_vertex(current_vertex,my_tasks);
		path_cost = compute_cost(ci);
		//printf("[Target NOT Set] Pathcost from %d to %d : %.2f \n",current_vertex,ci,path_cost);
	}
	//handle remaining locations in reverse utility order
	while (!all_selected(my_tasks)){
		int ni = return_next_vertex(ci,my_tasks);
		path_cost += compute_cost(ci,ni);
		//printf("[while loop] pathcost from %d to %d : %.2f \n",ci,ni,path_cost);
		ci=ni;
	}
	if (ci >= 0 && ci < dimension){
		printf("returning back from (last task) %d to (current vertex) %d (cost = %.2f) \n",ci,current_vertex,path_cost);
		path_cost += compute_cost(ci,current_vertex);
	}
	printf("## total cost = %.2f \n",path_cost);

	delete[] my_tasks;

	return path_cost;
}

void SSIPatrolAgent::force_bid(int nv,double bv,int rid){
	//printf("forcing bid for vertex %d with value %.2f from robot %d \n",nv,bv,rid);
	bids[nv].bidValue = bv;
	bids[nv].robotId = rid;
}

void SSIPatrolAgent::wait(){
	double t = std::min(timeout,1.0+nactivetasks*0.1);
#if DEBUG_PRINT
	printf("   --- waiting %.1f second ---\n",t);
#endif
	ros::Duration delay = ros::Duration(t); //asynchronous version
	delay.sleep();	

/*        double micro_timeout = timeout/ts; //synchronous version
	for (int i=0;i<ts;i++){
		// ros::spinOnce();
	        ros::Duration delay = ros::Duration(micro_timeout);
	        delay.sleep();	
	}
*/
}


int SSIPatrolAgent::compute_next_vertex() {
	compute_next_vertex(current_vertex);
}

// current_vertex (goal just reached)
int SSIPatrolAgent::compute_next_vertex(int cv) {

    update_global_idleness();


	//consider all possible vertices as next target (i.e., set all vertices to false)	
    reset_selected_vertices(selected_vertices);

	//consider only neighbouring vertices for next_vertex selection (i.e., set to false only neighbouring vertices)
	//NOTE: if none of the neighbouring vertices is assigned to the agent all other vertices will be considered (see reset_selected_vertices() called in select_next_vertex(...))
	//select_faraway_vertices(selected_vertices,cv);	
    if (cv >= 0 && cv < dimension){
        selected_vertices[cv] = true; //do not consider current vertex as possible goal 
    }

    int mnv = select_next_vertex(cv,selected_vertices);	
    double bidvalue = compute_bid(mnv); 
    int value = ID_ROBOT;
    if (value==-1){value=0;}
    force_bid(mnv,bidvalue,value); 
    send_target(mnv,bidvalue);
#if DEBUG_PRINT    
    printf("DTAP [%.1f] compute_next_vertex: waiting for bids\n",ros::Time::now().toSec());
#endif
    wait();
#if DEBUG_PRINT    
    printf("DTAP compute_next_vertex: bids timeout - current value for target node %d = %.2f \n",mnv,bidvalue);
#endif

    /*
    printf("Tasks [");
    for (size_t i = 0; i<dimension;i++){
		printf(" %d, ",tasks[i]);	
    }
    printf("] \n");
    */
    
    
    //printf("DTAP: while(true) ... \n");
    while (true){

    	if (best_bid(mnv)){ //if I am in the best position to go to mnv 
			update_tasks();
			//force_bid(mnv,0,value); TODO: check
			break;
    	} else {
		if (greedy_best_bid(cv,mnv)){ //if the greedy action condition is true stop the vertex selection and go to mvn (do not update your task)
#if DEBUG_PRINT
			//get date and time for file name
            time_t rawtime;
		    struct tm * timeinfo;
			char strnow[80];
			time (&rawtime);
			timeinfo = localtime(&rawtime);
			sprintf(strnow,"%d%02d%02d_%02d%02d%02d",  timeinfo->tm_year+1900,timeinfo->tm_mon+1,timeinfo->tm_mday,timeinfo->tm_hour,timeinfo->tm_min,timeinfo->tm_sec);
			//open file
 			FILE* fp = fopen("greedy-actions.txt","a");
			fprintf(fp,"time: %s; robot: %d; target vertex: %d; current vertex: %d\n",strnow,value,mnv,cv);
			fclose(fp);
#endif
			//exit from while loop	
			break;
		} else {
			mnv = select_next_vertex(cv,selected_vertices);	
			bidvalue = compute_bid(mnv); 
			force_bid(mnv,bidvalue,value); 
			send_target(mnv,bidvalue);
			//printf("  ... waiting for bids (%.2f seconds) ... \n",timeout);
			wait();
			/*printf("current target %d current value for target %.2f tasks [",mnv,bidvalue);
			    for (size_t i = 0; i<dimension;i++){
					printf(" %d, ",tasks[i]);	
			    }
			    printf("] \n");*/			
		}
	} 			
    }
    //printf("DTAP: while(true) ... DONE\n");
    
    return mnv;
    
}


// NOTE: redefined in DTASSIPart_Agent
void SSIPatrolAgent::update_tasks(){
    int value = ID_ROBOT;
    if (value==-1){value=0;}
    for (size_t i = 0; i< dimension; i++){
        tasks[i] = (bids[i].robotId == value);
    }
}

void SSIPatrolAgent::send_target(int nv,double bv) {
	//msg format: [ID_ROBOT,msg_type,next_vertex_index,bid_value]
        int value = ID_ROBOT;
        if (value==-1){value=0;}
    
    	int msg_type = DTASSI_TR;
    	std_msgs::Int16MultiArray msg;
    	msg.data.clear();

	msg.data.push_back(value);
        msg.data.push_back(msg_type);
        msg.data.push_back(nv);
#if DEBUG_PRINT
        printf("DTAP [%.1f]  ** sending Task Request [robot:%d, msgtype:%d, next_vertex:%d, bid:%.2f ] \n",
		ros::Time::now().toSec(),value,msg_type,nv,bv);
#endif
        int ibv = (int)(bv);
        if (ibv>32767) { // Int16 is used to send messages
            ROS_WARN("Wrong conversion when sending bid value in messages!!!");
            ibv=32000;
        }
	msg.data.push_back(ibv);
    	
	do_send_message(msg);   
    
}


void SSIPatrolAgent::send_bid(int nv,double bv) {
        int value = ID_ROBOT;
        if (value==-1){value=0;}
        
	//msg format: [ID_ROBOT,msg_type,next_vertex_index,bid_value]
    	int msg_type = DTASSI_BID;
    	std_msgs::Int16MultiArray msg;
    	msg.data.clear();

	msg.data.push_back(value);
        msg.data.push_back(msg_type);
        msg.data.push_back(nv);
#if DEBUG_PRINT
    	printf("DTAP  ** sending Bid [robot:%d, msgtype:%d, next_vertex:%d, bid:%.2f ] \n",value,msg_type,nv,bv);
#endif
        int ibv = (int)(bv);
        if (ibv>32767) { // Int16 is used to send messages
            ROS_WARN("Wrong conversion when sending bid value in messages!!!");
            ibv=32000;
        }
        msg.data.push_back(ibv);
	do_send_message(msg);   
}


//return true if the robot holds the best bid for nv OR if the vertex is adjacent on the patrol graph, the idleness is much higher than normal and no one else is going to the same vertex
bool SSIPatrolAgent::greedy_best_bid(int cv, int nv){


//	bool my_best = (bids[nv].robotId==ID_ROBOT);
//	printf("CHECK BEST: bid robot id %d, result: %d \n",bids[nv].robotId,(bids[nv].robotId==ID_ROBOT));


	bool adj = (compute_hops(cv,nv) <= 1);

	double avg_idleness = 0.;
	for(size_t i=0; i<dimension; i++) {
        	avg_idleness += global_instantaneous_idleness[i];
	}
	avg_idleness = avg_idleness/((double) dimension);
	double std_idleness = 0.;
	for(size_t i=0; i<dimension; i++) {
        std_idleness += (global_instantaneous_idleness[i] - avg_idleness)*(global_instantaneous_idleness[i] - avg_idleness);
	}
	std_idleness = sqrt(std_idleness/((double) dimension));
	bool high_idleness = (global_instantaneous_idleness[nv] > (2*std_idleness + avg_idleness));
	bool conflict = (bids[nv].bidValue == 0);
	bool greedy_cond = adj && high_idleness && !conflict;

#if DEBUG_PRINT
	printf("CHECK ADJ: cv %d, nv %d, hops: %d, result: %d \n",cv,nv,compute_hops(cv,nv),(int)adj);
	printf("CHECK HIGH IDLNESS: idl %f, avg %f, std %f, result: %d \n",global_instantaneous_idleness[nv],avg_idleness,std_idleness,high_idleness);
	printf("CHECK CONF: bid value %f, result: %d \n",bids[nv].bidValue,conflict);
	printf("CHECK GREEDY: result: %d \n",greedy_cond);
#endif

//	return my_best || greedy_cond;	
	return greedy_cond;
}


//return true if I have this robot holds the lowest bid for nv
bool SSIPatrolAgent::best_bid(int nv){
	// printf("computing whether I hold the best bid for %d, result: %d \n",nv,(bids[nv].robotId==ID_ROBOT));

/*    	printf("bids [");
        for (size_t i = 0; i<dimension; i++){
	    printf(" <%.2f,%d>, ",bids[i].bidValue,bids[i].robotId);	
        }
        printf("] \n"); 
*/
        int value = ID_ROBOT;
        if (value==-1){value=0;}
	return (bids[nv].robotId==value);	
}

// current_vertex (goal just reached)
// next_vertex (next goal)
//make this blocking to wait for bids
//
void SSIPatrolAgent::send_results() {
    int value = ID_ROBOT;
    if (value==-1){value=0;}
    
    //result= [ID,msg_type,global_idleness[1..dimension],next_vertex]
    int msg_type = DTAGREEDY_MSG_TYPE;
    std_msgs::Int16MultiArray msg;
    msg.data.clear();
    msg.data.push_back(value);
    msg.data.push_back(msg_type);
    // printf("  ** sending [%d, %d, ",ID_ROBOT,msg_type);
    pthread_mutex_lock(&lock);
    for(size_t i=0; i<dimension; i++) {
        // convert in 1/10 of secs (integer value) Max value 3276.8 second (> 50 minutes) !!!
        int ms = (int)(global_instantaneous_idleness[i]*10);
        if (ms>32767) { // Int16 is used to send messages
            ROS_WARN("Wrong conversion when sending idleness value in messages!!!");
            printf("*** idleness value = %.1f -> int16 value = %d\n",global_instantaneous_idleness[i],ms);
            ms=32000;
        }
//        if ((int)i==next_vertex) ms=0; //sending 0 for next vertex to avoid conflicts (useless for DTASSI) TODO:CHECK 
        //printf("  ** sending GII[%lu] = %d\n",i,ms);
        //printf("%d, ",ms);
        msg.data.push_back(ms);
    }
    pthread_mutex_unlock(&lock);
    msg.data.push_back(next_vertex);
    //printf("%d]\n",next_vertex);
    
    do_send_message(msg);   
}

void SSIPatrolAgent::update_bids(int nv, double bv, int senderId){
	if (bids[nv].bidValue >= (1 + hist)*bv) { //using histeresis to avoid switching when there is no clear benefit
		bids[nv].bidValue = bv;
		bids[nv].robotId = senderId;
	}
	update_tasks();
}

void SSIPatrolAgent::idleness_msg_handler(std::vector<int>::const_iterator it){

    double now = ros::Time::now().toSec();
    pthread_mutex_lock(&lock);
    for(size_t i=0; i<dimension; i++) {
		int ms = *it; it++; // received value
		// printf("  ** received from %d remote-GII[%lu] = %.1f\n",id_sender,i,ms);
		//printf("%d, ",ms);
		double rgi = (double)ms/10.0; // convert back in seconds
		global_instantaneous_idleness[i] = std::min(
			global_instantaneous_idleness[i]+(now-last_update_idl), rgi);
		// printf("   ++ GII[%lu] = %.1f (r=%.1f)\n",i,global_instantaneous_idleness[i],rgi);
    }
    pthread_mutex_unlock(&lock);
    last_update_idl = now;

/* TODO: CONSIDER INCLUDING THIS IN SSI AS A DIRECT DE-CONFLICTING PROCEDURE

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
*/
}

#if 0
void SSIPatrolAgent::task_request_msg_handler(std::vector<int>::const_iterator it, int senderId){
	int nv = *it; it++;
	double bv = *it; it++;
#if DEBUG_PRINT
	printf("DTAP handling task request message form %d: [ vertex: %d, bid value: %.2f]\n",senderId,nv,bv);
#endif
        double now = ros::Time::now().toSec();
	taskRequests[nv] = now;
	double my_bidValue = compute_bid(nv); 
	update_bids(nv,bv,senderId);
	if (my_bidValue<bv*(1+hist)){
		send_bid(nv,my_bidValue);
	}
}
#else
void SSIPatrolAgent::task_request_msg_handler(std::vector<int>::const_iterator it, int senderId){
        int nv = *it; it++;
        double bv = *it; it++;
        double now = ros::Time::now().toSec();
#if DEBUG_PRINT
	printf("DTAP [%.1f] handling task request message form %d: [ vertex: %d, bid value: %.2f]\n",now,senderId,nv,bv);
#endif

        taskRequests[nv] = now;

        update_bids(nv,bv,senderId);  // update bids with sender value

// CHECK: added Luca !!!
        //if (nv==next_vertex || nv==next_next_vertex) {
            double my_bidValue = compute_bid(nv);
            int value = ID_ROBOT;
            if (value==-1){value=0;}
	    update_bids(nv,my_bidValue,value);  // update bids with my value
	//}

//      if (my_bidValue<bv*(1+hist)){
        if (bids[nv].robotId==value){
//              send_bid(nv,my_bidValue);
                send_bid(nv,bids[nv].bidValue);
        }
}
#endif


void SSIPatrolAgent::bid_msg_handler(std::vector<int>::const_iterator it, int senderId){
	int nv = *it; it++;
	double bv = *it; it++;
#if DEBUG_PRINT
	printf("DTAP [%.1f] handling bid message from %d: [ vertex: %d, bid value: %.2f]\n",ros::Time::now().toSec(),senderId,nv,bv);
#endif
	update_bids(nv,bv,senderId);
}


void SSIPatrolAgent::receive_results() {
    //result= [ID,msg_type,global_idleness[1..dimension],next_vertex]
        
    std::vector<int>::const_iterator it = vresults.begin();
    int id_sender = *it; it++;
    int value = ID_ROBOT;
    if (value==-1){value=0;}
    if (id_sender==value) return;
    int msg_type = *it; it++;
    //printf("  ** received [%d, %d, ... ] \n",id_sender,msg_type);
    switch (msg_type){
	    case (DTAGREEDY_MSG_TYPE):
	    {
	        idleness_msg_handler(it);
	    }
	    break;
	    case (DTASSI_TR):
	    {
		task_request_msg_handler(it,id_sender);
	    }	
	    break;			    
	    case (DTASSI_BID):
	    {
		bid_msg_handler(it,id_sender);
	    }	
	    break;
   }
}




