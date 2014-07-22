#include "SSIPatrolAgent.h"

//#define MY_CONFIG_FILENAME "params/DTA/DTASSI.params"

//Sequential Single Item Auction with dynamic compact partition of the environment
class DTASSIPart_Agent: public SSIPatrolAgent {

protected:
    //center location given current tasks: task location that is at minimum path distance from all other locations. This is always a task location. 
    size_t current_center_location;		 

    //compute minimum path cost considering all tasks (tasks) and the next vertex (nv). 
    //The first room is always the current goal (if any), then rooms are visited in decreasing order of utility. 
    //The path cost is sum of travel cost given the order. 
    double compute_bid(int nv);	

    //update tasks seeting to true only the vertices for which this robot has the current highest bid
    //based on the array bids	
    void update_tasks();

    //compute center point given current tasks	
    void compute_center_location();	

    double compute_sum_distance(int cv);
	
	//compute number of hops to nv from cv
	size_t compute_hops(int cv, int nv);

	//compute utility considering number of hops instead of distance
	double utility(int cv,int nv);


public:

    DTASSIPart_Agent(){}	

    void init(int argc, char** argv);


};

void DTASSIPart_Agent::init(int argc, char** argv) {
    
//    logfile = fopen("DTASSIOut.log","w");	

//    fprintf(logfile,"INITIALIZING \n");
//    fflush(logfile);	

    SSIPatrolAgent::init(argc,argv);

//    fprintf(logfile,"INITIALIZING 2 \n");
//    fflush(logfile);	

    //set current center location to current vertex	    
    current_center_location = current_vertex;
	
//    fprintf(logfile,"initialised current center location to: %d \n",current_center_location);

    //initialize parameters

}

double DTASSIPart_Agent::compute_bid(int nv){

	printf("computing bid for vertex %d (using dynamic partition) \n ",nv);

	if (nv==next_vertex || nv==next_next_vertex){
		printf("already going to %d sending 0 (current target: %d)",nv,next_vertex);
		return 0.;
	}

	size_t num_tasks = 1;
    for (size_t i = 0; i<dimension ; i++){
		if (tasks[i]){
			num_tasks++;
		}
    }
	
//	size_t cv = current_vertex;
//	if (next_vertex >= 0 && next_vertex <dimension){
//		cv = next_vertex;
//	}
	double bid_value = compute_cost(nv,current_center_location)*num_tasks; 
	printf("bid for %d (current center %zu, num task %zu): %.2f \n",nv,current_center_location,num_tasks,bid_value);

	return bid_value;


}

size_t DTASSIPart_Agent::compute_hops(int cv, int nv)
{
    uint elem_s_path;
    int *shortest_path = new int[dimension]; 
    int id_neigh;
    
    dijkstra( cv, nv, shortest_path, elem_s_path, vertex_web, dimension); //structure with normal costs
    size_t hops = 0;
    
    for(uint j=0; j<elem_s_path; j++){
//        printf("path[%u] = %d\n",j,shortest_path[j]);
        
        if (j<elem_s_path-1){
            id_neigh = is_neigh(shortest_path[j], shortest_path[j+1], vertex_web, dimension);
			hops++;
        }       
    }
    
    return hops;
}        


double DTASSIPart_Agent::utility(int cv,int nv) {
    double idl = global_instantaneous_idleness[nv];
    size_t hops = compute_hops(cv,nv);
    double U = theta_idl * idl + theta_hop * hops;
    printf("  cv: %d -- U[%d] ( %.1f, %zu ) = %.1f\n",cv,nv,idl,hops,U);
    return U;
}

void DTASSIPart_Agent::compute_center_location(){
	size_t min = current_vertex;
//	printf("compute center:: min: %d current center: %d \n",min,current_center_location);
	double min_dist = compute_sum_distance(min);
//	printf("compute center:: min dist: %.2f \n",min_dist);
        for (size_t i = 0; i<dimension; i++){
		if (i!=current_center_location && tasks[i]){
			double dist = compute_sum_distance(i);
//			printf("compute center:: current dist: %.2f, min dist: %.2f, current min: %d, current point: %d \n",dist,min_dist,min,i);
			if ( dist < min_dist){
				min = i;
				min_dist = dist; 
			}
		}
        }
	current_center_location = min;
//	printf("current center: %d\n",current_center_location);
}

double DTASSIPart_Agent::compute_sum_distance(int cv){
	if(cv<0 || cv >= dimension){
//		printf("return big number: cv = %d",cv);
		return BIG_NUMBER;
	}
	double sum = 0.;
        for (size_t i = 0; i<dimension ; i++){
			if (tasks[i]){
	//			printf("sum: %2.f \n",sum);
				sum+= compute_cost(cv,i);
			}
        }
	return sum;
}

void DTASSIPart_Agent::update_tasks(){
    	
/*debug print
	printf("updating tasks: \n tasks before[");
        for (size_t i = 0; i<dimension; i++){
	    printf(" %d, ",tasks[i]);	
        }
        printf("] \n"); 

    	printf("bids [");
        for (size_t i = 0; i<dimension; i++){
	    printf(" <%.2f,%d>, ",bids[i].bidValue,bids[i].robotId);	
        }
        printf("] \n"); 

	printf("center location before %d \n",current_center_location);

------------*/


	bool changed = false;
	for (size_t i = 0; i< dimension; i++){
		if (!changed && tasks[i] != (bids[i].robotId == ID_ROBOT)){
			changed = true;		
		}
		tasks[i] = (bids[i].robotId == ID_ROBOT);
	}

/*debug print*/
    	printf("task after [");
        for (size_t i = 0; i<dimension; i++){
	    printf(" %d, ",tasks[i]);	
        }
        printf("] \n"); 

	printf("changed ? %d ",changed);
/*------------*/


	if (changed){
		compute_center_location();
	}

/*debug print*/

	printf("center location after %d \n",current_center_location);


/*------------*/
}



int main(int argc, char** argv) {
  
    DTASSIPart_Agent agent;
    agent.init(argc,argv);
    agent.run();

    return 0; 
}

