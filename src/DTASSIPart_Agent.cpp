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

	if (nv==next_vertex){
		printf("already going to %d sending 0 (current target: %d)",nv,next_vertex);
		return 0.;
	}

	double bid_value = compute_cost(nv,current_center_location);
	printf("bid for %d (current center %d): %.2f \n",nv,current_center_location,bid_value);

	return bid_value;


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

