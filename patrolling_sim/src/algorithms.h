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

typedef unsigned int uint;


typedef struct {
  int id, dist, elem_path;
  int path[1000];
  bool visit;
}s_path;

typedef struct {
  int id, elem_path;
  double dist;
  int path[1000];
  bool visit;
}s_path_mcost;

uint conscientious_reactive (uint current_vertex, vertex *vertex_web, double *instantaneous_idleness){

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
      neighbors[i] = vertex_web[current_vertex].id_neigh[i];		//neighbors table
      decision_table[i] = instantaneous_idleness [ neighbors[i] ];	//corresponding idleness table
      
      //choose the one with maximum idleness:
      if (decision_table[i] > max_idleness){
	max_idleness = decision_table[i];		//maximum idleness

	hits=0;
	possibilities[hits] = neighbors[i];
	
      }else if(decision_table[i] == max_idleness){
	hits ++;
	possibilities[hits] = neighbors[i];
      }
    }      
      
    if(hits>0){	//more than one possibility (choose at random)
      srand ( time(NULL) );
      i = rand() % (hits+1) + 0; 	//0, ... ,hits
	
      //printf("rand integer = %d\n", i);
      next_vertex = possibilities [i];		// random vertex with higher idleness
      	
      }else{
	next_vertex = possibilities[hits];	//vertex with higher idleness
      }
    
  }else{
    next_vertex = vertex_web[current_vertex].id_neigh[0]; //only one possibility
  }
  
  return next_vertex;

}


uint heuristic_conscientious_reactive (uint current_vertex, vertex *vertex_web, double *instantaneous_idleness){

  //number of neighbors of current vertex (number of existing possibilites)
  uint num_neighs = vertex_web[current_vertex].num_neigh;
  uint next_vertex;
  
  if (num_neighs > 1){
    
    //obtain max_idleness between neighbors
    //obtain max_distance between neighbors
    //build table with decision values (and neighbor index)
    
    uint i, max_distance=0;
    double max_idleness=-1;
    uint neighbors [num_neighs];    
    
    //printf("\n");
    
    //obtain max
    for (i=0; i<num_neighs; i++){      
      neighbors[i] = vertex_web[current_vertex].id_neigh[i];		//neighbors table

      if (instantaneous_idleness [neighbors[i]] > max_idleness){
	max_idleness = instantaneous_idleness [neighbors[i]];
      }
      
      if (vertex_web[current_vertex].cost[i] > max_distance){
	max_distance = vertex_web[current_vertex].cost[i];
      }
      
      //printf ("idleness[%u] = %f\n",neighbors[i],instantaneous_idleness [neighbors[i]]);
      //printf ("distance[%u] = %u\n",neighbors[i],vertex_web[current_vertex].cost[i]);
    }
    
    //printf ("\nmax idleness = %f\n", max_idleness);
    //printf ("max_distance = %u\n\n", max_distance);
    
    
    double normalized_idleness, normalized_distance, distance, idleness;
    double decision_table [num_neighs];
    
    //calculate decision values and build table
    for (i=0; i<num_neighs; i++){    
      
      distance = vertex_web[current_vertex].cost[i];
      idleness = instantaneous_idleness [ neighbors[i] ];
      
      normalized_distance = (double) (max_distance - distance) / (double) (max_distance); // [0,1]
      if (max_idleness == 0.0){
	normalized_idleness = 0.0;
      }else{
	normalized_idleness = idleness / max_idleness; // [0,1]
      }
     
      //printf ("normalized_distance[%u] = %f\n",neighbors[i],normalized_distance);
      //printf ("normalized_idleness[%u] = %f\n",neighbors[i],normalized_idleness);
      
      decision_table[i] = normalized_distance + normalized_idleness;     
      //printf("decision_table[%u] = %f\n\n",neighbors[i],decision_table[i]);
    }
      
    double max_decision=-1;
    uint hits = 0;
    uint possibilities[num_neighs];
    
    //verify if there is only one choice, if there are more - choose at random
    for (i=0; i<num_neighs; i++){  
      
      if (decision_table[i]>max_decision){
	  max_decision = decision_table[i];
	  hits = 0;
	  possibilities[hits] = neighbors[i];
      }else if (decision_table[i]==max_decision){
	  hits ++;
	  possibilities[hits] = neighbors[i];
      }      
    }
    
    if(hits>0){	//more than one possibility (choose at random)
      //printf("MORE THAN ONE POSSIBILITY, CHOOSE RANDOMLY\n");
      srand ( time(NULL) );
      i = rand() % (hits+1) + 0; 	//0, ... ,hits
	
      //printf("rand integer = %d\n", i);
      next_vertex = possibilities [i];		// random vertex with higher idleness
      //printf("next_vertex = %u\n",next_vertex);
      	
    }else{
	next_vertex = possibilities[hits];	//vertex with higher idleness
	//printf("next_vertex = %u\n",next_vertex);
    } 
  
  
  }else{ //num_neighs == 1 -> only one possible choice
    next_vertex = vertex_web[current_vertex].id_neigh[0]; //only one possibility
    //printf("Only 1 neighbor: next_vertex = %u\n",next_vertex);
  }
  
  return next_vertex;
  
}

uint greedy_bayesian_strategy (uint current_vertex, vertex *vertex_web, double *instantaneous_idleness, double G1, double G2, double edge_min){

  //number of neighbors of current vertex (number of existing possibilites)
  uint num_neighs = vertex_web[current_vertex].num_neigh;
  uint next_vertex;
  
  if (num_neighs > 1){
    
    double posterior_probability [num_neighs];
    uint neighbors [num_neighs];
    uint possibilities[num_neighs];
     
    uint i, hits=0;
    double max_pp= -1;
    double gain, exp_param, edge_weight;
    double log_result = log (1.0/G1);
    
    for (i=0; i<num_neighs; i++){
      neighbors[i] = vertex_web[current_vertex].id_neigh[i];		//neighbors table
     
      edge_weight = (double) vertex_web[current_vertex].cost[i];
      if (edge_weight<edge_min) {edge_weight = edge_min;}	
      
      //printf("Edge cost = %d\n",vertex_web[current_vertex].cost[i]);    
      gain = (instantaneous_idleness [ neighbors[i] ] / edge_weight);		//corresponding gain
      //printf("Gain = Inst Idleness / Edge Cost = %f / %f = %f\n", instantaneous_idleness [neighbors[i]],(double)vertex_web[current_vertex].cost[i],gain);
      
      if (gain < G2){
      
	//printf("Log_result = ln(1/G1) = 1.0 / %f = %f\n", G1, log_result);
	
	exp_param = (log_result/G2)*gain;
	//printf("exp_param = exp(log_param/G2 * gain) = ( %f / %f ) * %f = %f\n", log_param, (double)G2, gain, exp_param);
	
	posterior_probability[i] = G1 * exp (exp_param);	//P(move)=0.5 anula com P(gain) = 0.5 [factor de escala]
	//printf("posterior_probability = G1 * exp_param = %f  * %f = %f\n", G1, exp_param,  posterior_probability[i]);      
	
      }else{
	posterior_probability[i] = 1.0;
      }
      
      printf("Vertex [%d]; PP = %f\n", vertex_web[current_vertex].id_neigh[i], posterior_probability[i]);
      
      //choose the one with maximum posterior_probability:
      if (posterior_probability[i] > max_pp){
	max_pp = posterior_probability[i];		//maximum idleness

	hits=0;
	possibilities[hits] = neighbors[i];
	
      }else if (posterior_probability[i] == max_pp){
	hits ++;
	possibilities[hits] = neighbors[i];
      }
    }      
      
    if(hits>0){	//more than one possibility (choose at random)
      srand ( time(NULL) );
      i = rand() % (hits+1) + 0; 	//0, ... ,hits
	
      //printf("rand integer = %d\n", i);
      next_vertex = possibilities [i];		// random vertex with higher idleness
      	
      }else{
	next_vertex = possibilities[hits];	//vertex with higher idleness
      }
    
  }else{
    next_vertex = vertex_web[current_vertex].id_neigh[0]; //only one possibility
  }
  
  return next_vertex;

}

int count_intention (uint vertex, int *tab_intention, int nr_robots){
 
  int count = 0;
  
  for (int i = 0; i<nr_robots; i++){   
    if(tab_intention[i]==vertex){
      count++;      
    }    
  }
  return count;  
}

uint state_exchange_bayesian_strategy (uint current_vertex, vertex *vertex_web, double *instantaneous_idleness, int *tab_intention, int nr_robots, double G1, double G2, double edge_min){

  //number of neighbors of current vertex (number of existing possibilites)
  uint num_neighs = vertex_web[current_vertex].num_neigh;
  uint next_vertex;
  
  if (num_neighs > 1){
    
    double posterior_probability [num_neighs];
    uint neighbors [num_neighs];
    uint possibilities[num_neighs];
     
    uint i, hits=0;
    double max_pp= -1;
    double gain, exp_param, edge_weight;
    double log_result = log (1.0/G1);
    
    for (i=0; i<num_neighs; i++){
      neighbors[i] = vertex_web[current_vertex].id_neigh[i];		//neighbors table
     
      edge_weight = (double) vertex_web[current_vertex].cost[i];
      if (edge_weight<edge_min) {edge_weight = edge_min;}
      
      //printf("Edge cost = %d\n",vertex_web[current_vertex].cost[i]);    
      gain = (instantaneous_idleness [ neighbors[i] ] / edge_weight);		//corresponding gain
      //printf("Gain = Inst Idleness / Edge Cost = %f / %f = %f\n", instantaneous_idleness [neighbors[i]],(double)vertex_web[current_vertex].cost[i],gain);
      
      if (gain < G2){
      
	//printf("Log_result = ln(1/G1) = 1.0 / %f = %f\n", G1, log_result);
	
	exp_param = (log_result/G2)*gain;
	//printf("exp_param = exp(log_param/G2 * gain) = ( %f / %f ) * %f = %f\n", log_param, (double)G2, gain, exp_param);
	
	posterior_probability[i] = G1 * exp (exp_param);	//P(move)=0.5 anula com P(gain) = 0.5 [factor de escala]
	//printf("posterior_probability = G1 * exp_param = %f  * %f = %f\n", G1, exp_param,  posterior_probability[i]);      
	
      }else{
	posterior_probability[i] = 1.0;
      }
      
      //NESTE PONTO TEMOS A PP considerando so o GANHO, agora vamos considerar tb o ESTADO ("intenções") do sistema:
      
      //contar ocorrencias do vertice na tab_intention:
      int count = count_intention (neighbors[i], tab_intention, nr_robots);
      
      if (count>0){	//There is a robot who intends to go to this vertex! -> Update State
	
	//printf("COUNT = %d\n",count);
	printf("Vertex [%d]; PP (without state exchange) = %f\n", vertex_web[current_vertex].id_neigh[i], posterior_probability[i]);
	double P_gain_state = ( pow(2,nr_robots-count) ) / ( pow(2,nr_robots) - 1.0);	
	posterior_probability[i] *= P_gain_state;
	//printf("Vertex [%d]; PP (depois) = %f\n", vertex_web[current_vertex].id_neigh[i], posterior_probability[i]);
	
      }

      printf("Vertex [%d]; PP = %f\n", vertex_web[current_vertex].id_neigh[i], posterior_probability[i]);
      
      //choose the one with maximum posterior_probability:
      if (posterior_probability[i] > max_pp){
	max_pp = posterior_probability[i];		//maximum idleness

	hits=0;
	possibilities[hits] = neighbors[i];
	
      }else if (posterior_probability[i] == max_pp){
	hits ++;
	possibilities[hits] = neighbors[i];
      }
    }      
      
    if(hits>0){	//more than one possibility (choose at random)
      srand ( time(NULL) );
      i = rand() % (hits+1) + 0; 	//0, ... ,hits
	
      //printf("rand integer = %d\n", i);
      next_vertex = possibilities [i];		// random vertex with higher idleness
      	
      }else{
	next_vertex = possibilities[hits];	//vertex with higher idleness
      }
    
  }else{
    next_vertex = vertex_web[current_vertex].id_neigh[0]; //only one possibility
  }
  
  return next_vertex;

}

void dijkstra( uint source, uint destination, int *shortest_path, uint &elem_s_path, vertex *vertex_web, uint dimension){
  
  uint i,j,k,x;
  int id_next_vertex=-1;
  
  s_path *tab_dijkstra = new s_path[dimension];
  
  //Initialization:
  for(i=0; i<dimension; i++){
	
	tab_dijkstra[i].id = vertex_web[i].id;
	tab_dijkstra[i].elem_path = 0;
	tab_dijkstra[i].visit = false;
	
	if (vertex_web[i].id == source) {
	  tab_dijkstra[i].dist = 0;
	  tab_dijkstra[i].path[0] = source;
	  tab_dijkstra[i].elem_path++;
	  id_next_vertex = i;
	}else{
	  tab_dijkstra[i].dist = INT_MAX;
	}

  }  
  
  int next_vertex = source;
  int minim_dist;
  bool cont;
  
  while(true){	
	
//	printf("next_vertex = %i\n", next_vertex);
	
	if(next_vertex == destination){
	 break; 
	}
	
	tab_dijkstra[id_next_vertex].visit = true;
	
	/* id_next_vertex has INDEX of next_vertex in tab_dijkstra */
	/* j has INDEX of the neighbor in tab_disjkstra */
	/* k has index of the neighbor in the neighbor table of next_vertex */
	
	//Go to neihgobors;	
	for(k=0; k<vertex_web[next_vertex].num_neigh; k++){
	  
		cont = false;
		
		//condition - cannot have been visited:
		for(j=0; j<dimension; j++){
		  if(tab_dijkstra[j].id == vertex_web[next_vertex].id_neigh[k] && tab_dijkstra[j].visit == false){
			cont = true;
			break;
		  }		  
		}
		
		if(cont){
		  
		  //calculate distance from this neighbor:
		  if( tab_dijkstra[id_next_vertex].dist + vertex_web[next_vertex].cost[k] < tab_dijkstra[j].dist){
			
			//update distance to this vertex:
			tab_dijkstra[j].dist = tab_dijkstra[id_next_vertex].dist + vertex_web[next_vertex].cost[k];
			
			//update path (previous path + this one):
			for (x=0; x<tab_dijkstra[id_next_vertex].elem_path; x++){			 
			  tab_dijkstra[j].path[x] = tab_dijkstra[id_next_vertex].path[x];			  
			}
			
			tab_dijkstra[j].path[tab_dijkstra[id_next_vertex].elem_path] = tab_dijkstra[j].id;
			tab_dijkstra[j].elem_path = tab_dijkstra[id_next_vertex].elem_path+1;

		  }	
		  
		}
		

 
	}
	
	minim_dist = INT_MAX;
	
	//decide next_vertex:
	for(i=0; i<dimension; i++){	  
	  
	  if(tab_dijkstra[i].dist < minim_dist && tab_dijkstra[i].visit == false){
		minim_dist = tab_dijkstra[i].dist;
		next_vertex = tab_dijkstra[i].id;
		id_next_vertex = i;
	  }	 
	  
	}
	
  }
  
  //Save shortest_path & delete tab_dijkstra... 
  elem_s_path = tab_dijkstra[id_next_vertex].elem_path; //id_next_vertex has the ID of the destination in tab_dijkstra

  for(i=0; i<elem_s_path; i++){	
	shortest_path[i] = tab_dijkstra[id_next_vertex].path[i];	
  }
  
  delete [] tab_dijkstra;
  
}


int is_neigh(uint vertex1, uint vertex2, vertex *vertex_web, uint dimension){
  int i;
  
  for (i=0; i<vertex_web[vertex1].num_neigh; i++){
   if(vertex2 == vertex_web[vertex1].id_neigh[i]){
     return i; //neighbor_id (vertex1 in respect to vertex2)
   }
  }
  
  return -1; //not neighbor
}


void dijkstra_mcost( uint source, uint destination, int *shortest_path, uint &elem_s_path, vertex *vertex_web, double new_costs[][8], uint dimension){
  
  uint i,j,k,x;
  int id_next_vertex=-1;
  
  s_path_mcost *tab_dijkstra = new s_path_mcost[dimension];
  
  //Initialization:
  for(i=0; i<dimension; i++){
	
	tab_dijkstra[i].id = vertex_web[i].id;
	tab_dijkstra[i].elem_path = 0;
	tab_dijkstra[i].visit = false;
	
	if (vertex_web[i].id == source) {
	  tab_dijkstra[i].dist = 0.0;
	  tab_dijkstra[i].path[0] = source;
	  tab_dijkstra[i].elem_path++;
	  id_next_vertex = i;
	}else{
	  tab_dijkstra[i].dist = INFINITY;
	}

  }  
  
  int next_vertex = source;
  double minim_dist;
  bool cont;
  
  while(true){	
	
//	printf("next_vertex = %i\n", next_vertex);
	
	if(next_vertex == destination){
	 break; 
	}
	
	tab_dijkstra[id_next_vertex].visit = true;
	
	/* id_next_vertex has INDEX of next_vertex in tab_dijkstra */
	/* j has INDEX of the neighbor in tab_disjkstra */
	/* k has index of the neighbor in the neighbor table of next_vertex */
	
	//Go to neihgobors;	
	for(k=0; k<vertex_web[next_vertex].num_neigh; k++){
	  
		cont = false;
		
		//condition - cannot have been visited:
		for(j=0; j<dimension; j++){
		  if(tab_dijkstra[j].id == vertex_web[next_vertex].id_neigh[k] && tab_dijkstra[j].visit == false){
			cont = true;
			break;
		  }		  
		}
		
		if(cont){
		  
		  //calculate distance from this neighbor:
		  if( tab_dijkstra[id_next_vertex].dist + new_costs[next_vertex][k] < tab_dijkstra[j].dist){
			
			//update distance to this vertex:
			tab_dijkstra[j].dist = tab_dijkstra[id_next_vertex].dist + new_costs[next_vertex][k];
			
			//update path (previous path + this one):
			for (x=0; x<tab_dijkstra[id_next_vertex].elem_path; x++){			 
			  tab_dijkstra[j].path[x] = tab_dijkstra[id_next_vertex].path[x];			  
			}
			
			tab_dijkstra[j].path[tab_dijkstra[id_next_vertex].elem_path] = tab_dijkstra[j].id;
			tab_dijkstra[j].elem_path = tab_dijkstra[id_next_vertex].elem_path+1;

		  }	
		  
		}
		

 
	}
	
	minim_dist = INFINITY;
	
	//decide next_vertex:
	for(i=0; i<dimension; i++){	  
	  
	  if(tab_dijkstra[i].dist < minim_dist && tab_dijkstra[i].visit == false){
		minim_dist = tab_dijkstra[i].dist;
		next_vertex = tab_dijkstra[i].id;
		id_next_vertex = i;
	  }	 
	  
	}
	
  }
  
  //Save shortest_path & delete tab_dijkstra... 
  elem_s_path = tab_dijkstra[id_next_vertex].elem_path; //id_next_vertex has the ID of the destination in tab_dijkstra

  for(i=0; i<elem_s_path; i++){	
	shortest_path[i] = tab_dijkstra[id_next_vertex].path[i];	
  }
  
  delete [] tab_dijkstra;
  
}


uint heuristic_pathfinder_conscientious_cognitive (uint current_vertex, vertex *vertex_web, double *instantaneous_idleness, uint dimension, uint *path){
  
	  //Heuristic Decision (Considering ALL vertices of the graph and shortest path to them)
	  uint distance[dimension]; 			//distance from current_vertex to all vertices	  
	  uint i,j, elem_s_path, max_distance=0, max_cost=0, min_cost=INT_MAX;
	  double max_idleness=-1;
	  
	  
	  for (i=0; i<dimension; i++){
	    
	    //max_idleness
	    if (instantaneous_idleness [i] > max_idleness){
	      max_idleness = instantaneous_idleness [i];
	    }      
	   
	    if (i==current_vertex){	      
	      distance[i] = 0;
// 	      printf("distance[%u]=%u\n\n",i,distance[i]);
	      
	    }else{
	      
	      int id_neigh = is_neigh(current_vertex, i, vertex_web, dimension);
	      
	      if (id_neigh>=0){	//neighbors		
		distance[i] = vertex_web[current_vertex].cost[id_neigh];
// 		printf("distance[%u]=%u\n\n",i,distance[i]);
		
	      }else{	//not neighbors
		
		int *shortest_path = new int[dimension]; 
		uint j;
	
		//Call with normal costs:
		dijkstra( current_vertex, i, shortest_path, elem_s_path, vertex_web, dimension); //structure with normal costs
		distance[i] = 0;
		
		for(j=0; j<elem_s_path; j++){
// 		  printf("caminho[%u] = %d\n",j,shortest_path[j]);
		  
		  if (j<elem_s_path-1){
		    id_neigh = is_neigh(shortest_path[j], shortest_path[j+1], vertex_web, dimension);
		    distance[i] += vertex_web[shortest_path[j]].cost[id_neigh];
		  }		  
		}
// 		printf("distance[%u]=%u\n\n",i,distance[i]);
		delete [] shortest_path;
	      }	      
	    }
	    
	    //max and min costs:
	    for(j=0; j<vertex_web[i].num_neigh; j++){
	      if (vertex_web[i].cost[j] > max_cost){
		max_cost = vertex_web[i].cost[j];
	      }	    
	      if (vertex_web[i].cost[j] < min_cost){
		min_cost = vertex_web[i].cost[j];
	      }
	    }    
	    
	    //max distance:
	    if (distance[i] > max_distance){
	      max_distance = distance[i];
	    }	    
	  }
	  
// 	  printf("max_distance = %u\n\n",max_distance);
// 	  printf("max_cost = %u\n\n",max_cost);
// 	  printf("min_cost = %u\n\n",min_cost);
	  
	  double normalized_idleness, normalized_distance;
	  double decision_table [dimension];
	  
	  //obtain max_decision having max_distance & max_idleness
	  for (i=0; i<dimension; i++){    
	    
	    normalized_distance = (double) (max_distance - distance[i]) / (double) (max_distance); // [0,1]
	    
	    if (max_idleness == 0.0){
	      normalized_idleness = 0.0;
	    }else{
	      normalized_idleness = instantaneous_idleness[i] / max_idleness; // [0,1]
	    }
	    	    
	    if (i!=current_vertex){
	      decision_table[i] = normalized_distance + normalized_idleness;     
	    }else{
	      decision_table[i] = 0.0;
	    }
	  }  
	  
// 	  printf("max_idleness = %f\n\n",max_idleness);
	  
	  //next_vertex is the one with max_decision
// 	  double max_decision = 0.0;
	  uint next_vertex;
	  
	  //Pathfinder -> Convert the weights (edge costs) of the graph to a new scale and find shortest path to the next_vertex
	  double new_costs [dimension][8];
		  
	  for (i=0; i<dimension; i++){
	    
	    //Get Next Vertex:
//  	    printf("decision_table[%u] = %f\n",i,decision_table[i]);
	    
	   /* if (decision_table[i]>max_decision){
	     next_vertex = i;
	     max_decision = decision_table[i]; // GUARDAR PARA TABELA DE POSSIBILIDADES E ESCOLHER ALEATORIAMENTE
	    }*/
	    
	    //Get New Edge Costs:
	    for(j=0; j<vertex_web[i].num_neigh; j++){
		  
		  //the same edge will have 2 different costs depending on the direction of travelling:
		  if (max_idleness==0.0){
		    normalized_idleness = 0.0;
		  }else{
		    normalized_idleness = (max_idleness - instantaneous_idleness[ vertex_web[i].id_neigh[j] ]) / max_idleness; 
		  }
		  
		  if (max_cost==min_cost){
		    normalized_distance = 0.0;
		  }else{
		    normalized_distance = (double)(vertex_web[i].cost[j] - min_cost) / (double)(max_cost-min_cost);
		  }
		  
// 		  printf("normalized_idleness = %f\n", normalized_idleness);
// 		  printf("normalized_distance = %f\n", normalized_distance);
		  
		  new_costs[i][j] = normalized_idleness + normalized_distance;	
		  
//  		  printf("new_costs[%d][%d] = %f\n\n", i,j,new_costs[i][j]);
	    }
	    
	  }
	  
	  
	double max_decision=-1;
	uint hits = 0;
	uint possibilities[dimension];
	
	//verify if there is only one choice, if there are more - choose at random
	for (i=0; i<dimension; i++){  
		
		if (decision_table[i]>max_decision){
			max_decision = decision_table[i];
			hits = 0;
			possibilities[hits] = i;
		}else if (decision_table[i]==max_decision){
			hits ++;
			possibilities[hits] = i;
		}      
	}
	
	if(hits>0){	//more than one possibility (choose at random)
		//printf("MORE THAN ONE POSSIBILITY, CHOOSE RANDOMLY\n");
		srand ( time(NULL) );
		i = rand() % (hits+1) + 0; 	//0, ... ,hits
			
		//printf("rand integer = %d\n", i);
		next_vertex = possibilities [i];		// random vertex with higher idleness
		//printf("next_vertex = %u\n",next_vertex);
			
	}else{
		next_vertex = possibilities[hits];	//vertex with higher idleness
		//printf("next_vertex = %u\n",next_vertex);
	} 	  
	  
//  	  printf("max_decision=%f\n\n",max_decision);
//  	  printf("next_vertex=%u\n",next_vertex);
	  
	  int *shortest_path = new int[dimension]; 
  
  	  //Disjkstra function with modified costs:
	  dijkstra_mcost( current_vertex, next_vertex, shortest_path, elem_s_path, vertex_web, new_costs, dimension);
	  
	  //int caminho_final[elem_s_path];
	  
// 	  printf("\n");
	  for(i=0; i<elem_s_path; i++){
	    path[i] = shortest_path[i];
// 	    printf("path [%d] = %d\n",i,path[i]);
	  }
	  
	  delete [] shortest_path;
	  
	  return elem_s_path;
  
}

//Check if an element belongs to a table
bool pertence (int elemento, int *tab, int tam_tab){
  
  for (int i=0; i<tam_tab; i++){
	
	if( tab[i] == elemento) {
	  return true;
	}
	
  }
  
  return false;  
}

int pertence_idx (int elemento, int *tab, int tam_tab){
  
  for (int i=0; i<tam_tab; i++){
	
	if( tab[i] == elemento) {
	  return i;
	}
	
  }
  
  return -1;  
}

bool UHC (vertex *vertex_web, int v1, int *caminho_principal, uint dimension) {
  
//  printf("\n\nORIGEM_da_PROCURA=%i\n",v1);
//  printf("1o.Vertice = %i\n", v1);
  
  int prox_vertice = v1, elem_cp = 0, v, i, i_viz;
  int caminho_parcial [dimension];
  int *viz_pvert = new int[8];
  srand(1000);
  
  caminho_parcial[elem_cp] = prox_vertice;
  elem_cp++;
//  printf("adicionei prox_vertice=%i ao caminho parcial\n", prox_vertice);
  
  
  while( elem_cp < dimension ){
	
	i_viz=0;
	
	//construir tab de vizinhos do prox_vertice:
	for(i=0; i<vertex_web[prox_vertice].num_neigh; i++){
		
	  if ( !pertence (vertex_web[prox_vertice].id_neigh[i], caminho_parcial, elem_cp) ){
	    viz_pvert[i_viz] = vertex_web[prox_vertice].id_neigh[i];
	    i_viz++;
	  }

	  
	}	
	
	if(i_viz>0){
	  i = rand() % i_viz;
	  v = viz_pvert[i]; //v = vizinho aleatorio do prox_vertice
	}else{
//	  printf("Todos os vizinhos ja pertencem ao caminho parcial\n");
	  return false;	  
	}
	
//	printf("v = vizinho aleatorio do prox_vertice = %i\n",v);
//	printf("elem_cp = %i\t dimension = %i\n", elem_cp, dimension);
	
	
	if( !pertence(v, caminho_parcial, elem_cp) ) { 		//adicionar v ao caminho parcial:
//		printf("adicionar v = %i ao caminho parcial\n",v);
		caminho_parcial [elem_cp] = v;
		elem_cp++;
//		printf("prox_vertice = %i\n", v);
		prox_vertice = v;
	}
	
/*	printf("\nCAMINHO PARCIAL ATE AO MOMENTO:\n");
	
	for(i=0; i<elem_cp; i++){
	  printf("caminho_parcial[%i]=%i\n", i, caminho_parcial[i]); 
	}
*/	
  }
  
//  printf("ENCONTREI C. HAMILTON!\n");
  
  delete [] viz_pvert;
  
  //atribuir caminho principal:
  for(i=0; i<elem_cp; i++){
	caminho_principal[i] = caminho_parcial[i];
  }
  
  return true;  
  
}

void clear_visited (vertex *vertex_web, uint dimension){ //LIMPA TODA A REDE DE NOS (NAO SO DA REGIAO)
  
  int i,j;
  
  for (i=0; i<dimension; i++){
	for(j=0; j<vertex_web[i].num_neigh; j++){
	  //inicializar a false:
	  vertex_web[i].visited[j] = false;
	}
  }
  
}

bool procurar_ciclo (vertex *vertex_web, uint dimension, int *caminho_principal, int &elem_caminho, int &custo_max, int seed) {
  
  int i, k=0, n, rec, i_list=0, origem, custo, ult_elemento, elem_cp=0, caminho_parcial [dimension];
  bool encontra_destino, escolha_aleatoria;
  int *lista_v2v = new int[1000];
  
  clear_visited(vertex_web, dimension);

  for(i=0; i<dimension; i++){
	
	if( vertex_web[i].num_neigh > 1 ) { //tem + d 1 vizinho: criar tabela com estes.
	  
	  lista_v2v[i_list] = vertex_web[i].id;
	  i_list++; 
	  
	}
  }



  bool encontrou_ciclo=false;
  int count=0;
  custo_max = 0;

  for (n=0; n<i_list+1; n++){
	
	if(encontrou_ciclo && count<3){ //numero de tentativas total apos encontrar o 1� ciclo = 3
		
		count++;
		seed+=1000;
		n--; //volta a repetir o mm 'n' com outra semente

	}else{
	  count = 0;
	  seed = 1000;
	  if(n==i_list){break;}
	}
	
	encontrou_ciclo = false;	
	srand(seed);

	
	//limpar tudo:
	clear_visited(vertex_web, dimension);
	
	
	//colocar nos com 1 so vizinho a TRUE [e reciprocos!!]:
	for(i=0; i<dimension; i++){
	  
	  if( vertex_web[i].num_neigh == 1 ) { //so tem 1 vizinho
		
		vertex_web[i].visited[0] = true;
		//	  printf(" Arco de (%i para %i).visitado = true\n", elem_reg[i], vertex_web[elem_reg[i]].id_neigh[0] );
		
		rec = vertex_web[i].id_neigh[0];
		
		for(k=0; k<vertex_web[rec].num_neigh; k++){
		  if (vertex_web[rec].id_neigh[k] ==  vertex_web[i].id){
			vertex_web[rec].visited[k]=true;
			//		  printf(" Arco de (%i para %i).visitado = true\n", rec, vertex_web[rec].id_neigh[k] );
			break;
		  }
		}
		
	  }
	}
  
	
//	printf("\n\n========================== NO %i ============================\n\n", lista_v2v[n]);
	
	custo = 0;
	origem = lista_v2v[n];
	elem_cp = 0;	
	encontra_destino = false; //numa primeira fase queremos adiar a chegada ao destino.
	int *viz = new int[8];
	ult_elemento=-1;
	int aux, conta, viz_c_4_arcos;
	
	//caminho:
	caminho_parcial [ elem_cp ] = origem;
	elem_cp++;
	
	while( ult_elemento!=origem || elem_cp>1 ){ //acaba quando voltar a origem.
	  
	  escolha_aleatoria = false;
	  
	  ult_elemento = caminho_parcial [elem_cp-1];
	  
//	  printf("---------------------------------------\n");
//	  printf("Ultimo Elemento = %i\n", ult_elemento);
//	  printf("num. de elementos no cp= %i\n", elem_cp);
//	  printf("origem= %i\n", origem);
	  
	  //verificar estado dos vizinhos do elemento actual:	
	  k=0;
	  
	  //construir tab de vizinhos do prox_vertice:
	  for(i=0; i<vertex_web[ult_elemento].num_neigh; i++){
		
		//pertence a mm regiao:
		//if (vertex_web[vertex_web[ult_elemento].id_neigh[i]].regiao == ID_REG || (vertex_web[ult_elemento].regiao == ID_REG && vertex_web[vertex_web[ult_elemento].id_neigh[i]].regiao != ID_REG && vertex_web[ult_elemento].arco_front[i] == true) ){
		//if( pertence( vertex_web[ult_elemento].id_neigh[i], elem_reg, dimension) ){
		  
		  //nunca foi visitado e nao pertence ao caminho parcial:
		  //if ( vertex_web[ult_elemento].visitado[i] == false && ( !pertence(vertex_web[ult_elemento].id_neigh[i], caminho_parcial, elem_cp) || vertex_web[ult_elemento].id_neigh[i] == origem) ){
		  
		  /*pode repetir vertices: na pratica so o vai fazer s este tiver 4 ou + vizinhos*/
		  if ( vertex_web[ult_elemento].visited[i] == false){ 
			viz[k] = vertex_web[ult_elemento].id_neigh[i];
			k++;
		  }
		//}
		
	  }
	  
	  
	  if (k==0){ //ja nao ha vizinhos disponiveis -- voltar para tras.
		
//		printf("k=0\n");
		encontra_destino = true;
		
		if(elem_cp>1){
		  //voltar para o anterior e passar visitados a false (menos o q liga este vizinho ao anterior no caminho parcial.
//		  printf("percorrer vizinhos de %i\n", vertex_web[ult_elemento].id);
		 
		  for(i=0; i<vertex_web[ult_elemento].num_neigh; i++){
			
//			printf("vizinho %i\n", vertex_web[ult_elemento].id_neigh[i]);
			
			//if (vertex_web[vertex_web[ult_elemento].id_neigh[i]].regiao == ID_REG || (vertex_web[ult_elemento].regiao == ID_REG && vertex_web[vertex_web[ult_elemento].id_neigh[i]].regiao != ID_REG && vertex_web[ult_elemento].arco_front[i] == true) ){
			//if( pertence( vertex_web[ult_elemento].id_neigh[i], elem_reg, dimension) ){	
			  
//			  printf("%i pertence a mesma regiao\n", vertex_web[ult_elemento].id_neigh[i]);
			  
			  //vizinhos com + d 1vizinho:
			  if ( vertex_web[ vertex_web[ult_elemento].id_neigh[i] ].num_neigh > 1 ){
				
//				printf("%i tem + de 1 vizinho\n", vertex_web[ult_elemento].id_neigh[i]);

				//NAO ESTA NO CAMINHO PARCIAL:
				if( !pertence(vertex_web[ult_elemento].id_neigh[i],caminho_parcial,elem_cp)){
   
//				  printf("%i e diferente do ult. vertice do caminho parcial %i\n", vertex_web[ult_elemento].id_neigh[i], vertex_web[caminho_parcial[elem_cp-2]].id);
   
				  //passar para false:
				  vertex_web[ult_elemento].visited[i] = false;   
//				  printf("ID = %i, Vizinho = %i -> disponivel\n", vertex_web[ult_elemento].id, vertex_web[ult_elemento].id_neigh[i]);
   
				/*Reciproco: */
				
				for(k=0; k<vertex_web[vertex_web[ult_elemento].id_neigh[i]].num_neigh; k++){
				  if( vertex_web[vertex_web[ult_elemento].id_neigh[i]].id_neigh[k] ==  vertex_web[ult_elemento].id){
					vertex_web[vertex_web[ult_elemento].id_neigh[i]].visited[k] = false;
//					printf("Reciproco: ID = %i, Vizinho = %i -> descartado\n", vertex_web[vertex_web[ult_elemento].id_neigh[i]].id, vertex_web[vertex_web[ult_elemento].id_neigh[i]].id_neigh[k]);
					break;
				  }
				}
				
			  }
			}
		  //}		  
		}
		
		  elem_cp--; //retirar o ult.elemento (nao volta a ser escolhido pq o camiho ta true para ele).
		  
		}else{
		  //1 ou 0 e n tem vizinhos disponiveis
//		  printf("so tem 1 elemento no caminho parcial e nao ha caminho pro destino. \n");
		  break;
		}
		
	  }else{
		
//		printf("k>0\n");
		
		//se tem a origem como vizinho;
		if(pertence(origem, viz, k)){
		  
//		 printf("tem a origem como vizinho!\n");
		  
		  //encontrei origem!!!!
		  if(encontra_destino || k==1){
			
			  /* FINAL- encontrei origem! */
			caminho_parcial[elem_cp] = origem;
			elem_cp++;
//			printf("acabei com sucesso (nao sobram + vizinhos para alem da origem)\n");
			
/*			printf("caminho_parcial: ");
			for(i=0; i<elem_cp; i++){
			  if(i==elem_cp-1){printf("%i\n", caminho_parcial[i]);}else{printf("%i, ", caminho_parcial[i]);}
			}
			printf("elementos = %i\n----------------------\n", elem_cp);
*/

				
			/* calcular custo do caminho parcial: */
			//elementos em caminho parcial:
			
			for (i=0; i<elem_cp-1; i++){			
			  for(k=0; k<vertex_web[ caminho_parcial[i] ].num_neigh; k++){			  
				if( vertex_web[ caminho_parcial[i] ].id_neigh[k] == vertex_web[ caminho_parcial[i+1] ].id ){				
				  custo += vertex_web[ caminho_parcial[i] ].cost[k];
				  break;
				}				
			  }			
			}
			
//			printf("custo = %i\n", custo);
			
			if(custo>custo_max){
			  custo_max=custo;
			  
			  /* IR GUARDANDO O MELHOR...*/
			  for(i=0; i<elem_cp; i++){
				caminho_principal[i] = caminho_parcial[i];
			  }
			  
			  elem_caminho = elem_cp;
			}
			
			
			//return true;
			encontrou_ciclo = true;
			break;
		  
		  }else{ //retirar o destino das hipoteses (k>1)
		  
//			printf("retirar a origem da lista de vizinhos!\n");
			
			for(i=0; i<k; i++){
			  
			  if(viz[i] == origem){
				
				if(i<k-1){
				  for(rec=i; rec<k-1; rec++){
					viz[rec] = viz[rec+1];				  
				  }
				}
				
				//diminuir elementos:
				k--;
				
			  }			
			}
			
/*			printf("lista de vizinhos apos retirar:\n");
			for(i=0; i<k; i++){
			  printf("viz[%i]=%i\n",i,viz[i]);
			}
*/				  
			/* escolher aleatoriamente proximo elemento: */		 
			escolha_aleatoria = true;
		  }
		  
		}else{
		  /* escolher aleatoriamente proximo elemento: */		 
		  escolha_aleatoria = true;
		}
		
		if(escolha_aleatoria == true){
		  
		  if(encontra_destino){/*printf("passei encontra destino a false\n");*/encontra_destino=false;}
		  
		  /* escolher aleatoriamente proximo elemento:
		  
		  A NAO SER QUE UM DOS VIZINHOS TENHA PELO MENOS 4 ARCOS 
		  LIGADO A ELE QUE AINDA NAO TENHAM SIDO VISITADOS -> nesse caso escolher esse, 
		  caso contrario: ALEATORIO. */
		  
		  //variavel inteira com vizinho nessa situaçao (inicializada a -1):
		  viz_c_4_arcos = -1;
		  
		  for(i=0; i<k; i++){
			
			conta=0;
			
//			printf("verificar arcos ligados a %i\n", viz[i]);
			
			for(aux=0; aux<vertex_web[viz[i]].num_neigh; aux++){
			  
//			  printf("arco q liga a %i\n", vertex_web[viz[i]].id_neigh[aux]);			  
			  
			 // if (vertex_web[vertex_web[viz[i]].id_neigh[aux]].regiao == ID_REG || (vertex_web[viz[i]].regiao == ID_REG && vertex_web[vertex_web[viz[i]].id_neigh[aux]].regiao != ID_REG && vertex_web[viz[i]].arco_front[aux] == true) ){
				
//				printf("ta na mm regiao\n");
				
				if(vertex_web[viz[i]].visited[aux] == false){
				  conta++;
//				  printf("conta = %i\n",conta);
				}
				
				if(conta>=4){
				  viz_c_4_arcos = viz[i];
//				  printf("encontrei vizinho com 4 arcos livres: %i\n", viz[i]);
				  break;
				}
				
			  //}
			  
			}
		  }
		  
		  
		  //CASO +COMUM: ESCOLHER ALEATORIAMENTE;
		  if (viz_c_4_arcos==-1){
			i = rand() % k;
			caminho_parcial [elem_cp] = viz[i];
			
			//CASO EM Q VIZINHO TEM 4 ARCOS ADJACENTES LIVRES - escolhe-lo:
		  }else{
			caminho_parcial [elem_cp] = viz_c_4_arcos;
		  }
		  
		  
/*		  printf("escolhas possiveis: ");
		  for(i=0; i<k; i++){printf("%i ",viz[i]);}
		  printf("\nproximo vertice escolhido aleatoriamente = %i\n", caminho_parcial [elem_cp]);
*/		  
		  
		  //passar o arco entre eles para true.
		  for(rec=0; rec<vertex_web[ult_elemento].num_neigh; rec++){
			if( is_neigh ( vertex_web[ult_elemento].id, vertex_web[ult_elemento].id_neigh[rec], vertex_web, dimension ) > -1){			  
			  
			  if (vertex_web[ult_elemento].id_neigh[rec] == caminho_parcial [elem_cp]){
//				printf("arco de %i para %i passou a fazer parte do caminho\n", vertex_web[ult_elemento].id, vertex_web[ult_elemento].id_neigh[rec]);
				vertex_web[ult_elemento].visited[rec] = true;				
				
				
				//vertex_web de ID=caminho_parcial [elem_cp]
				//passe a ter vizinho = vertex_web[ult_elemento].id com visitado a true.
				
				for(k=0; k<vertex_web[caminho_parcial [elem_cp]].num_neigh; k++){
				  if (vertex_web[caminho_parcial [elem_cp]].id_neigh[k] == vertex_web[ult_elemento].id){
//					printf("RECIPROCO: arco de %i para %i passou a fazer parte do caminho\n", vertex_web[caminho_parcial [elem_cp]].id, vertex_web[caminho_parcial [elem_cp]].id_neigh[k]);
					vertex_web[caminho_parcial [elem_cp]].visited[k]=true;
					break;
				  }
				}	 
				
			  }	
			  
			}
		  }
		  
		  elem_cp++;
		  
	  }
	  
	}
	  
	  
	  
	
	
	}//fechar WHILE
	
	delete [] viz;
//	printf("---------------------------------------\n");
	
  }//for
  
//  if(custo_max>0){printf("custo maximo do ciclo = %i\n",custo_max);
//  }else{printf("nao possui ciclo.\n");}
  return true;
  
}

int devolve_viz_unicos (vertex *vertex_web, int vertice){
  
  if (vertex_web[vertice].num_neigh == 1){
	return vertex_web[vertice].id_neigh[0];
	
  }else{
	return -1;
  }
  
}

int longest_path (vertex *vertex_web, int origem, int destino, int *lista_v1v, int i_list, int dimension, int seed, int *caminho_parcial, int &elem_cp) {
  
//  printf("ORIGEM - No %i\tDESTINO - No %i\n", origem, destino);
  
  //PENSAR SEMPRE NO RECIPROCO!!!!!!!!!!!
  bool debug = false; //DEBUGGING PURPOSES
  
  int i, k;
  int rec;
  //int caminho_parcial [num_nos_regiao];
  /*int*/ elem_cp = 0;
  int custo = 0;
  
  //limpar tudo
  clear_visited(vertex_web, dimension);

  
  for(i=0; i<i_list; i++){
	if(!(lista_v1v[i]== origem || lista_v1v[i]==destino)){
	  
	  vertex_web[ lista_v1v[i] ].visited[0] = true;
	  if(debug){printf("ID = %i, Vizinho = %i -> descartado\n", lista_v1v[i],vertex_web[ lista_v1v[i] ].id_neigh[0]);}
	  
	  rec = vertex_web[ lista_v1v[i] ].id_neigh[0];
	  
	  for(k=0; k<vertex_web[rec].num_neigh; k++){
		if (vertex_web[rec].id_neigh[k] == vertex_web[lista_v1v[i]].id){
		  vertex_web[rec].visited[k]=true;
		  break;
		  if(debug){printf("ID = %i, Vizinho = %i -> descartado\n", rec,vertex_web[ rec ].id_neigh[k]);}
		}
	  }
	
	}else{ //apenas para garantir q a origem e o destino estao a FALSE:
	  
	  vertex_web[ lista_v1v[i] ].visited[0] = false;
	  
	  rec = vertex_web[ lista_v1v[i] ].id_neigh[0];
	  
	  for(k=0; k<vertex_web[rec].num_neigh; k++){
		if (vertex_web[rec].id_neigh[k] == vertex_web[lista_v1v[i]].id){
		  vertex_web[rec].visited[k]=false;
		  break;
		}
	  }
	  
	}
  }
  
  if(debug){printf("\n");}
  
  
  
  bool encontra_destino = false; //numa primeira fase queremos adiar a chegada ao destino.
  bool escolha_aleatoria;
  int *viz = new int[8];
  int ult_elemento=-1;
  int aux, conta, viz_c_4_arcos;
  //caminho:
  caminho_parcial [ elem_cp ] = origem;
  elem_cp++;
  
  srand(seed);

  
  while( ult_elemento!=destino ){ //acaba quando chegar ao destino.
	
	escolha_aleatoria = false;

	ult_elemento = caminho_parcial [elem_cp-1];

	if(debug){printf("---------------------------------------\n");
	printf("Ultimo Elemento = %i\n", ult_elemento);
	printf("elementos no cp= %i\n", elem_cp);}
	
	//verificar estado dos vizinhos do elemento actual:	
	k=0;
	
	//construir tab de vizinhos do prox_vertice:
	for(i=0; i<vertex_web[ult_elemento].num_neigh; i++){
	  
	  //if (vertex_web[vertex_web[ult_elemento].id_neigh[i]].regiao == ID_REG || (vertex_web[ult_elemento].regiao == ID_REG && vertex_web[vertex_web[ult_elemento].id_neigh[i]].regiao != ID_REG && vertex_web[ult_elemento].arco_front[i] == true) ){
	  //if( pertence( vertex_web[ult_elemento].id_neigh[i], elem_reg, num_nos_regiao) ){
		
		/* basta nao ter sido visited (para cobrir o caso de ter 4 ou + vizinhos */
		if ( vertex_web[ult_elemento].visited[i] == false /*&& !pertence(vertex_web[ult_elemento].id_neigh[i], caminho_parcial, elem_cp)*/){
		  viz[k] = vertex_web[ult_elemento].id_neigh[i];
		  k++;
		}
	  //}
	  
	}
	
	if(debug){printf("k=%i\n",k);}
	

	if (k==0){ //ja nao ha vizinhos disponiveis -- voltar para tras.
	  encontra_destino = true;
	  
	  if(elem_cp>1){
		//voltar para o anterior e passar visiteds a false (menos o q liga este vizinho ao anterior no caminho parcial.
		if(debug){printf("percorrer vizinhos de %i\n", vertex_web[ult_elemento].id);}
		for(i=0; i<vertex_web[ult_elemento].num_neigh; i++){
		  
		  if(debug){printf("vizinho %i\n", vertex_web[ult_elemento].id_neigh[i]);}
		  
		  //if (vertex_web[vertex_web[ult_elemento].id_neigh[i]].regiao == ID_REG || (vertex_web[ult_elemento].regiao == ID_REG && vertex_web[vertex_web[ult_elemento].id_neigh[i]].regiao != ID_REG && vertex_web[ult_elemento].arco_front[i] == true) ){
		  //if( pertence( vertex_web[ult_elemento].id_neigh[i], elem_reg, num_nos_regiao) ){	 
			
// 			printf("%i pertence a mesma regiao\n", vertex_web[ult_elemento].id_neigh[i]);
			
			//vizinhos com + d 1vizinho:
			if ( vertex_web[ vertex_web[ult_elemento].id_neigh[i] ].num_neigh  > 1 ){
			  
			 if(debug){ printf("%i tem + de 1 vizinho\n", vertex_web[ult_elemento].id_neigh[i]); }

			  //diferentes do ultimo arco do caminho parcial:
			  //if( vertex_web[ult_elemento].id_neigh[i] != vertex_web[caminho_parcial[elem_cp-2]].id){
			  //NAO ESTA NO CAMINHO PARCIAL:
			  if( !pertence(vertex_web[ult_elemento].id_neigh[i],caminho_parcial,elem_cp)) {
				
				if(debug){ printf("%i e diferente do ult. vertice do caminho parcial %i\n", vertex_web[ult_elemento].id_neigh[i], vertex_web[caminho_parcial[elem_cp-2]].id); }
				
				//passar para false:
				vertex_web[ult_elemento].visited[i] = false;		
				if(debug){ printf("ID = %i, Vizinho = %i -> disponivel\n", vertex_web[ult_elemento].id, vertex_web[ult_elemento].id_neigh[i]); }
			  
				/*Reciproco: */
				//ID = vertex_web[ult_elemento].id_neigh[i]
				//ID_VIZ = vertex_web[ult_elemento].id
				
				for(k=0; k<vertex_web[vertex_web[ult_elemento].id_neigh[i]].num_neigh; k++){
				  if( vertex_web[vertex_web[ult_elemento].id_neigh[i]].id_neigh[k] ==  vertex_web[ult_elemento].id){
					vertex_web[vertex_web[ult_elemento].id_neigh[i]].visited[k] = false;
					if(debug){ printf("Reciproco: ID = %i, Vizinho = %i -> descartado\n", vertex_web[vertex_web[ult_elemento].id_neigh[i]].id, vertex_web[vertex_web[ult_elemento].id_neigh[i]].id_neigh[k]);}
					break;
				  }
				}
  
			  }
			}
		  //}		  
		}
		elem_cp--; //retirar o ult.elemento (nao volta a ser escolhido pq o camiho ta true para ele).
	  }else{
		//1 ou 0 e n tem vizinhos disponiveis
//		printf("so tem 1 elemento no caminho parcial e nao ha caminho pro destino (Erro?) \n");
		return 0;
	  }
	  
	}else{
	  
	  //se tem o destino como vizinho;
	  if(pertence(destino, viz, k)){
		
//		printf("tem o destino como vizinho!\n");
		
		//encontrei destino!!!!
		if(encontra_destino || k==1){
		  
		  /* FINAL- encontrei destino! */
		  caminho_parcial[elem_cp] = destino;
		  elem_cp++;
		if(debug){  printf("acabei com sucesso (nao sobram + vizinhos para alem do destino)\n");
		  printf("caminho_parcial: ");
		  for(i=0; i<elem_cp; i++){
			if(i==elem_cp-1){printf("%i\n", caminho_parcial[i]);}else{printf("%i, ", caminho_parcial[i]);}
		  }
		  printf("elementos = %i\n----------------------\n", elem_cp); }
		  
		  /* calcular custo do caminho parcial: */
		  //elementos em caminho parcial:
		  
		  for (i=0; i<elem_cp-1; i++){			
			for(k=0; k<vertex_web[ caminho_parcial[i] ].num_neigh; k++){			  
			  if( vertex_web[ caminho_parcial[i] ].id_neigh[k] == vertex_web[ caminho_parcial[i+1] ].id ){				
				custo += vertex_web[ caminho_parcial[i] ].cost[k];
				break;
			  }				
			}			
		  }
		  
//		  printf("custo = %i\n", custo);

		  delete [] viz;
		  return custo;
		
		}else{ //retirar o destino das hipoteses (k>1)
		
		if(debug){printf("retirar o destino da lista de vizinhos!\n");}
		  
		  for(i=0; i<k; i++){
			
			if(viz[i] == destino){
			  
			  if(i<k-1){
				for(rec=i; rec<k-1; rec++){
				  viz[rec] = viz[rec+1];				  
				}
			  }
			  
			  //diminuir elementos:
			  k--;
			  
			}			
		  }
		  
		  if(debug){printf("lista de vizinhos apos retirar:\n");
		  for(i=0; i<k; i++){printf("viz[%i]=%i\n",i,viz[i]); }
		  }
		  
		  /* escolher aleatoriamente proximo elemento: */		 
		  escolha_aleatoria = true;
		}
		
	  }else{
		/* escolher aleatoriamente proximo elemento: */		 
		escolha_aleatoria = true;
	  }
	  
	  if(escolha_aleatoria == true){
	  
		if(encontra_destino){/*printf("passei encontra destino a false\n");*/encontra_destino=false;}
		
		  /* escolher aleatoriamente proximo elemento:
		  
		  A NAO SER QUE UM DOS VIZINHOS TENHA PELO MENOS 4 ARCOS 
		  LIGADO A ELE QUE AINDA NAO TENHAM SIDO VISITADOS -> nesse caso escolher esse, 
		  caso contrario: ALEATORIO. */
		  
		  //variavel inteira com vizinho nessa situaçao (inicializada a -1):
		  viz_c_4_arcos = -1;
		  
		  for(i=0; i<k; i++){
			
			conta=0;
			
			if(debug){printf("verificar arcos ligados a %i\n", viz[i]);}
			
			for(aux=0; aux<vertex_web[viz[i]].num_neigh; aux++){
			  
			  if(debug){printf("arco q liga a %i\n", vertex_web[viz[i]].id_neigh[aux]);}			  
			  
			  //if (vertex_web[vertex_web[viz[i]].id_neigh[aux]].regiao == ID_REG || (vertex_web[viz[i]].regiao == ID_REG && vertex_web[vertex_web[viz[i]].id_neigh[aux]].regiao != ID_REG && vertex_web[viz[i]].arco_front[aux] == true) ){
				
// 				printf("ta na mm regiao\n");
				
				if(vertex_web[viz[i]].visited[aux] == false){
				  conta++;
				  if(debug){printf("conta = %i\n",conta);}
				}
				
				if(conta>=4){
				  viz_c_4_arcos = viz[i];
				  if(debug){printf("encontrei vizinho com 4 arcos livres: %i\n", viz[i]);}
				  break;
				}
				
			  //}
			  
			}
		  }
		  
		  
		  //CASO +COMUM: ESCOLHER ALEATORIAMENTE;
		  if (viz_c_4_arcos==-1){
			i = rand() % k;
			caminho_parcial [elem_cp] = viz[i];
			
		  //CASO EM Q VIZINHO TEM 4 ARCOS ADJACENTES LIVRES - escolhe-lo:
		  }else{
			caminho_parcial [elem_cp] = viz_c_4_arcos;
		  }
		  
		  
		 if(debug){ printf("escolhas possiveis: ");
		  for(i=0; i<k; i++){printf("%i ",viz[i]);}
		 printf("\nproximo vertice escolhido aleatoriamente = %i\n", caminho_parcial [elem_cp]); }
		  
		  //passar o arco entre eles para true.
		  for(rec=0; rec<vertex_web[ult_elemento].num_neigh; rec++){
			if( is_neigh ( vertex_web[ult_elemento].id, vertex_web[ult_elemento].id_neigh[rec], vertex_web, dimension ) > -1){			  
			  
			  if (vertex_web[ult_elemento].id_neigh[rec] == caminho_parcial [elem_cp]){
				if(debug){ printf("arco de %i para %i passou a fazer parte do caminho\n", vertex_web[ult_elemento].id, vertex_web[ult_elemento].id_neigh[rec]);}
				vertex_web[ult_elemento].visited[rec] = true;				
			  	
			  
				//vertex_web de ID=caminho_parcial [elem_cp]
				//passe a ter vizinho = vertex_web[ult_elemento].id com visited a true.
				
				for(k=0; k<vertex_web[caminho_parcial [elem_cp]].num_neigh; k++){
				  if (vertex_web[caminho_parcial [elem_cp]].id_neigh[k] == vertex_web[ult_elemento].id){
					if(debug){ printf("RECIPROCO: arco de %i para %i passou a fazer parte do caminho\n", vertex_web[caminho_parcial [elem_cp]].id, vertex_web[caminho_parcial [elem_cp]].id_neigh[k]);}
					vertex_web[caminho_parcial [elem_cp]].visited[k]=true;
					break;
				  }
				}	 
			  
			  }	
			  
			}
		  }
		  
		  elem_cp++;

	  }

	}
	
  }
  
  delete [] viz;
  return 0;
  
}

bool caminho_apartir_vizinhos_unicos (vertex *vertex_web, int dimension, int *caminho_principal, int &elem_max, int &custo_max, int seed){
  
  int i,j,i_uc = 0;
  
  int *lista_v1v = new int[1000];
  int i_list=0;
  

	/* CRIAR LISTA DOS VIZINHOS UNICOS: */
	int valor = -1;
//	int pertence_ = -1;
	
	for(i=0; i<dimension; i++){
	  
	  valor = devolve_viz_unicos (vertex_web, vertex_web[i].id);
	  
	  if (valor > -1){ 	//no so tem 1 vizinho
		  
		  lista_v1v[i_list] = vertex_web[i].id;
// 		  printf("LISTA_vizinhos_unicos [%i] = %i\n", i_list, lista_v1v[i_list]);
		  i_list++;

	  }
	  
	}

	
	//se mesmo assim i_list <= 1 -> e pq todos os nos tem 2 ou + vizs...
	if (i_list<=1){
	  printf("Impossible to compute longest path.\n");
//	  printf("Nao existem nos so com 1 vizinho. Impossivel computar caminho +longo\n");
	  return false;
	}
	
//	delete [] tab_viz_unic;
//	delete [] tab_viz_comuns;
	
	
	/* PERCORRER TODOS OS NOS ENTRE SI PARA ENCONTRAR MAIOR CAMINHO */
	custo_max = 0; 
	int retorno=0, origem_caminho=-1, destino_caminho=-1, elem_caminho = 0;
	int caminho_parcial[dimension];
	
	for(i=0; i<i_list; i++){
	  for(j=0; j<i_list; j++){
		if(i<j){
			//encontrar longest_path entre vizinhos unicos...
			retorno = longest_path(vertex_web, lista_v1v[i], lista_v1v[j], lista_v1v, i_list, dimension, seed, caminho_parcial, elem_caminho);
			
			if(retorno>custo_max){
				custo_max=retorno;
				
				for(i_uc = 0; i_uc<elem_caminho; i_uc++){
				  caminho_principal[i_uc] = caminho_parcial[i_uc];
				}
				
				elem_max = elem_caminho;
				
				origem_caminho=lista_v1v[i];
				destino_caminho=lista_v1v[j]; 
			  }
		}
	  }
	}
	
//	printf("custo maximo do caminho = %i\n",custo_max);
//	printf("origem = %i\tdestino = %i\n", origem_caminho, destino_caminho);
	delete [] lista_v1v;
	return true;
  
}

bool verificar_arco_cp (int no_1, int no_2, int *caminho_principal, int elem_cp){
  
  int i;
  for(i=0; i<elem_cp; i++){
	
	if(caminho_principal[i] == no_1){
	 
	  if(i>0){
		if(caminho_principal[i-1] == no_2){
		 return true; 
		}
	  }
	  
	  if(i<elem_cp-1){
		if(caminho_principal[i+1] == no_2){
		  return true; 
		}
	  }
	  
	}	
  }
  
  return false;
  
}

bool check_visited (vertex *vertex_web, int dimension){
 
  int i, j;
  
  for(i=0; i<dimension; i++){
	
	for(j=0; j<vertex_web[i].num_neigh; j++){
	 
		if( vertex_web[i].visited[j] == false ){		  

		  return false;
		  
		}  
	}	
  }
  
  return true;
  
}


int devolve_vizinhos_nao_visitados (vertex *vertex_web, int dimension, int vertice){
  
  int i_viz=0;
  int i;
  
  //contar vizinhos do vertice da mesma regiao:
  for(i=0; i<vertex_web[vertice].num_neigh; i++){
	
//	if (vertex_web[vertex_web[vertice].id_viz[i]].regiao == ID_REG || (vertex_web[vertice].regiao == ID_REG && vertex_web[tab_nos[vertice].id_viz[i]].regiao != ID_REG && vertex_web[vertice].arco_front[i] == true) ){
	  
	  if(vertex_web[vertice].visited[i] == false){
	  	i_viz++;
	  }
	//}
 
  }
	
	return i_viz;
	
  }

bool computar_caminho_de_ida (vertex *vertex_web, int dimension, int *caminho_de_ida, int &elem_c_ida, int *caminho_principal, int elem_cp, int num_arcos){
  
  int i,j;
  int rec, idx_viz_cp=-1;
//  int num_arcos=49;				//nº de arcos q pdemos visitar no caminho secundario
  int arcos_ja_visitados;
//  int elem_c_ida=0;				//guarda registo do nº de elementos do caminho de ida
  int desvio [dimension*5]; 			//para ir mantendo nos a percorrer qd entramos no caminho secundario
  int regresso [dimension];
  int i_desvio=0, i_reg, idx_reg_directo=-1;
  bool sai_logo;
  int no_ant, prox_no, ult_no_cp, viz_escolhido, idx_cp=0;
  bool sai, tem_reg_directo, pertence_CP;
  int lista_viz[8];
  int i_list, num_neigh;
  
  caminho_de_ida[0] = caminho_principal[idx_cp];
//  printf("caminho de ida [%i] = %i\n", elem_c_ida, caminho_de_ida[elem_c_ida]);
  elem_c_ida++;
  
  while(idx_cp<elem_cp){
	
	
	//inicializa-lo ao proximo elemento do caminho principal
	prox_no = caminho_principal[idx_cp+1];
	ult_no_cp = caminho_principal[idx_cp];
	
	//proteccao:
	if(vertex_web[ult_no_cp].id != ult_no_cp){vertex_web[ult_no_cp].id = ult_no_cp;} 
	
	//decidir qual e o prox no:
	for(j=0; j<vertex_web[ult_no_cp].num_neigh; j++){
	   //sao vizinhos e na mm regiao:
	   if( is_neigh (ult_no_cp, vertex_web[ult_no_cp].id_neigh[j],vertex_web,dimension) > -1 //verificar se sao vizinhos
		 && vertex_web[ult_no_cp].visited[j] == false  							//ainda nao foram visitados
		 && !verificar_arco_cp (ult_no_cp, vertex_web[ult_no_cp].id_neigh[j], caminho_principal, elem_cp) ){ //o arco nao pertence ao caminho principal
		  
		  prox_no = vertex_web[ult_no_cp].id_neigh[j];
//		  printf("vou-me desviar em %i\n", prox_no);
		  break;
		}
		
		//se for igual ao proximo no no caminho - guardar o index
		if (vertex_web[ult_no_cp].id_neigh[j]== caminho_principal[idx_cp+1]){
		  idx_viz_cp = j;
		}
		
	 }
	 
//	 printf("prox_no = %i\n", prox_no);
//	 printf("idx_viz_cp = %i (tabela de %i p viz. no c.principal)\n", idx_viz_cp, ult_no_cp);
	 
	 //mantem-se no caminho principal:
	 if(prox_no == caminho_principal[idx_cp+1]){
	   
		//ASSINALAR ARCO VISITADO:
		for(i=0; i<vertex_web[ult_no_cp].num_neigh; i++){
		  if(vertex_web[ult_no_cp].id_neigh[i] == vertex_web[ult_no_cp].id_neigh[idx_viz_cp] && vertex_web[ult_no_cp].visited[i] == false) {
			vertex_web[ult_no_cp].visited[i] = true;		
		  }
		}
		
		rec = vertex_web[ult_no_cp].id_neigh[idx_viz_cp];
		//reciproco:
		for(j=0; j<vertex_web[rec].num_neigh; j++){
		  if(vertex_web[rec].id_neigh[j] == ult_no_cp && vertex_web[rec].visited[j]==false){
			vertex_web[rec].visited[j] = true;
		  }
		}	
		
		//adicionar proximo no do caminho principal ao caminho de ida (arcos a TRUE - visitados)
		caminho_de_ida[elem_c_ida] = prox_no;
//		printf("caminho de ida [%i] = %i\n", elem_c_ida, caminho_de_ida[elem_c_ida]);
		elem_c_ida++;
		idx_cp++;
		
		if(prox_no == caminho_principal[elem_cp-1]){
//		  printf("TERMINAMOS!\n");
		  break;
		}
	   
	 }else{ //desvio apartir do prox_no -> caminho secundario:
		/* encontrar desvio completo - assinalar arcos - passar para caminho de ida e voltar */

	   //index j -> tem o index do vizinho na tab do no anterior.
//	   printf("desvio apartir de %i\n", vertex_web[ult_no_cp].id_neigh[j] );	   
//	   printf("ultimo no do cp: %i\n", ult_no_cp);
	   
	   /*	   
	   de acordo com o valor de num_arcos, construir a tabela desvio q começa e acaba em ult_no_cp	 
	   dar preferencia a nos com vizinhos unicos
	   */

	  arcos_ja_visitados = 0;
	  no_ant = ult_no_cp;
	  //prox_no = vertex_web[ult_no_cp].id_neigh[j];
	  //j guarda o index.
	  //desvio [num_nos_regiao]; 	//para ir mantendo nos a percorrer qd entramos no caminho secundario
	  //regresso [num_nos_regiao];
	  i_desvio=0; i_reg=0;
	  desvio[i_desvio] = ult_no_cp;
	  i_desvio++;
	  bool assinala;
	  sai = false;
	  sai_logo = false;
	  pertence_CP = false; //GARANTIR REGRESSO AO NO ANTERIOR QUANDO E TRUE.
	  
	  
	  if( pertence (vertex_web[ult_no_cp].id_neigh[j], caminho_principal, elem_cp) ){
//		printf("ESTE PERTENCE JA AO CAMINHO PRINCIPAL!\n");
		pertence_CP = true;
		sai = true;
	  }
	  
	  while (arcos_ja_visitados<num_arcos){ //desvio IDA -> construir tabela de desvio e regresso
		
		tem_reg_directo = false;
		assinala = false;
		
		//assinala no:
		//vertex_web[no_ant].visited[j] = true;
		
		/* j tem o index do prox_no na tabela de vizinhos de no_ant - so e assinalado qd for necessario */
		
		//assinala reciproco:
		for(i=0; i<vertex_web[prox_no].num_neigh; i++){
		  if(vertex_web[prox_no].id_neigh[i] == no_ant && vertex_web[prox_no].visited[i] == false){ //protecçao contra loopings
			vertex_web[prox_no].visited[i] = true;
			assinala = true;
//			printf("assinalei arco %i - %i\n", prox_no, vertex_web[prox_no].id_neigh[i]);
		  }
		  
		  //verficar se tem vizinho directo para ult_no_cp.
		  if (vertex_web[prox_no].id_neigh[i] == ult_no_cp){
			tem_reg_directo = true;
			idx_reg_directo = i;
		  }
		}
		  
		//so assinala se forem, de facto, vizinhos:
		if(assinala){
			
		  //protecçao contra loopings:
		  for(i=0; i<vertex_web[no_ant].num_neigh; i++){			  
			if(vertex_web[no_ant].id_neigh[i] == vertex_web[no_ant].id_neigh[j] && vertex_web[no_ant].visited[i] == false){
			  vertex_web[no_ant].visited[i] = true; 		  
//			  printf("assinalei arco %i - %i\n", no_ant, vertex_web[no_ant].id_neigh[i]);
			}
		  }
		}
		  

		
		desvio[i_desvio] = prox_no;

		//no que ja existe no desvio - cortar caminho ja existente de regresso:
		viz_escolhido = pertence_idx (desvio[i_desvio], desvio, i_desvio-1);
		
		if( viz_escolhido > -1){ 
//		  printf("no %i ja pertence ao desvio - nao adicionar no anterior ao regresso\n", desvio[i_desvio]);

		  //pode ser um ciclo - neste caso:
		  //no anterior so se adicionar ao regresso se o ultimo elemento de regresso nao for vizinho do prox_no.
		  
		  if ( is_neigh (prox_no, regresso[i_reg-1], vertex_web, dimension) == -1 ){ //não são vizinhos
			
			regresso[i_reg] = no_ant;
			i_reg++;
			
		  }
		  
		}else{//no novo no desvio - caminho de regresso para ele apartir do ult no:  
		  
		  regresso[i_reg] = no_ant;
		  i_reg++;		  
		  
		  //so conta para arco visitado quando o no e novo:
		  arcos_ja_visitados++;
		}
				
		//incrementar desvio (foi adicionado no):
		i_desvio++;
		
		//ultimo no pertence ao caminho principal - vamos forçar o regresso ao no anterior:
		if(pertence_CP){
		  desvio[i_desvio] = regresso[i_reg-1];
		  i_desvio++;
		  i_reg--;
		  
		  //trocar prox_no com no_ant:
		  rec = no_ant;
		  no_ant = prox_no;
		  prox_no = rec;		  
		}
		
		
/*		printf("TABELA DE DESVIO:\n");
		for(i=0; i<i_desvio; i++){
		  printf("desvio[%i]=%i:\n",i,desvio[i]);
		}	
		
		printf("TABELA DE REGRESSO:\n");
		for(i=0; i<i_reg; i++){
		  printf("regresso[%i]=%i:\n",i,regresso[i]);
		}
*/		

	
		//inicializar:
		pertence_CP = false;
		
		viz_escolhido = -1;
		
		if(arcos_ja_visitados>=num_arcos){ //tomar regresso ou ver s ha no directo deste para ult_no_cp
		  
//		  printf("arcos_ja_visitados>=num_arcos -> vamos regressar ou tomar arco directo\n");
		  
		  if (tem_reg_directo && i_reg>0){ //tem regresso directo:
			
//			printf("%i tem regresso directo por %i\n", prox_no, vertex_web[prox_no].id_neigh[idx_reg_directo]);
			
			//assinalar arco:
			vertex_web[prox_no].visited[idx_reg_directo] = true;
			
			//assinalar reciproco:
			for(i=0; i<vertex_web[ult_no_cp].num_neigh; i++){
			  if(vertex_web[ult_no_cp].id_neigh[i] == prox_no){
				vertex_web[ult_no_cp].visited[i] = true;
			  }
			}

			desvio[i_desvio] = ult_no_cp;
			i_desvio++;
			
//			printf("TABELA FINAL DE DESVIO:\n");
//			for(i=0; i<i_desvio; i++){
//			  printf("desvio[%i]=%i:\n",i,desvio[i]);
//			}
			
			break; //sai do while - desvio completo.
			
		  }else{ //acrescentar o regresso:

			sai=true;
		  }
		  
		}else{ //ainda posso continuar (se puder)
		
//		  printf("arcos_ja_visitados < num_arcos -> vamos continuar a procura de arcos novos\n");
		
		  //criar lista de vizinhos.
		  i_list=0;
		  pertence_CP = false;
				  
		  for(i=0; i<vertex_web[prox_no].num_neigh; i++){
			
			if (is_neigh (prox_no, vertex_web[prox_no].id_neigh[i], vertex_web, dimension)>-1  //verificar se sao vizinhos da mm regiao
			  && vertex_web[prox_no].visited[i] == false && !verificar_arco_cp(prox_no, vertex_web[prox_no].id_neigh[i], caminho_principal, elem_cp) ){ //arco ainda nao foi visitado e o ARCO nao pertence ao CP
			  
			  lista_viz[i_list] = vertex_web[prox_no].id_neigh[i];
			  i_list++;
			  
			}
		  }
		  
//		  printf("lista de vizinhos de %i:", prox_no);
//		  for(i=0; i<i_list; i++){
//			printf(" %i",lista_viz[i]);
//		  }
		  
		  i=0;
		  //se so tiver 1 nao entra.
		  while(i_list>1){
			
			if( pertence(lista_viz[i], caminho_principal, elem_cp)){			  
			  //retirar vizinho da lista:
			  if(i<i_list-1){
				for(j=i; j<i_list-1; j++){
				  lista_viz[j] = lista_viz[j+1];				  
				}
			  }				  
			  //diminuir elementos:
			  i_list--;
			}
			
			i++;
			
			if(i == i_list){ //ha mais de 1 vizinho e nenhum pertence ao CP (ou so sobrou 1 viz):
			  
//			  printf("\nlista apos retirar arco(s) q pertence(m) ao CP:");
//			  for(i=0; i<i_list; i++){
//				printf(" %i",lista_viz[i]);
//			  }			  
			  
			  break;
			}
			
		  }
		  
		  if(i_list==1){
//			printf("\nlista so tem 1 vizinho\n");
			if(pertence(lista_viz[0], caminho_principal, elem_cp)){
			 pertence_CP = true; 
//			 printf("no vizinho pertence ao caminho principal - temos d garantir q regressamos na prox iteracao\n");
			}
		  }
		  
		  if(i_list <= 0){ //nao tem vizinhos - volta para tras
			
//			printf("\nlista de vizinhos vazia - %i n tem vizinhos validos... voltar para tras\n", prox_no);
			
			//voltar para tras ate encontrar elemento q tenha vizinhos (ou xegar ao fim)
			for(i=i_reg-1; i>=0; i--){			 
			  
			  j = devolve_vizinhos_nao_visitados(vertex_web, dimension, regresso[i]);
			  
			  //verificar se ha vizinhos livres ou se se continua.
			  if( j > 0) {
//				if(i!=0){printf("%i (membro de regresso) ainda tem vizinhos livres\n", regresso[i]); }
				//ha vizinhos livres - paro neste no.
				break;
			  
			  }else{
				//adicionar ao desvio (retirar do regresso):
				desvio[i_desvio] = regresso [i];
				i_desvio++;
				i_reg--;
			  }
			  
			}
			
			if(i<=0){ //regressamos ao ult_no_cp
//			    printf("nenhum membro de regresso tem vizinhos livres...regressamos ao ultimo elemento no CP\n");
			  //equivalente a adicionar regresso (i_reg inda tem o valor certo)
			  
			  sai = true;
			  
			}else{//tamos agora noutro no.
			  
			  //encontrar equivalente a "i" (regresso) no desvio.
			  for(j=0; j<i_desvio;j++){	  
				if (desvio[j] == regresso[i]){
				  prox_no = desvio[j];
				  no_ant = desvio[j+1];
//				  printf("prox_no = %i\tno_ant = %i\n", prox_no, no_ant);
				  break;
				}
				
				if(j==i_desvio-1){
//				  printf("ERRO: nao houve atribuicao de prox_no e no_ant\n");
				  printf("Error: There was no atribution of previous and next vertex.\n");
				}
			  }			  
			  
			  //decrementar elementos de regresso:
//			  printf("decrementar i_reg de %i para %i\n", i_reg, i_reg-(i_reg-i) );
			  i_reg-=(i_reg-i);
			  

//			  printf("TABELA DE REGRESSO:\n");
//			  for(i=0; i<i_reg; i++){
//				printf("regresso[%i]=%i:\n",i,regresso[i]);
//			  }  
			  
			  
			}
			
			
		  }else{ //decidir viz_escolhido
			
//			printf("\nvamos decidir qual e o proximo vizinho\n");
			
			if(i_list==1){
			 viz_escolhido = lista_viz[0]; 
//			 printf("viz_escolhido = %i\n", viz_escolhido);
			 
			}else{
			  //escolher o no q tem menos vizinhos.
			  
//			  printf("escolher no q tem menos vizinhos\n");
			  
			  num_neigh=INT_MAX;
			  
			  for(i=0; i<i_list; i++){
				
				j=devolve_vizinhos_nao_visitados(vertex_web, dimension, lista_viz[i]);
//				printf("{no %i} - vizinhos nao visitados= %i\n", lista_viz[i], j);
				
				if(j<num_neigh){
				  viz_escolhido = lista_viz[i];
				  num_neigh = j;
				}
				
				/* mandar backup fora */
				/*int k,l;
				for(k=0; k<num_nos_total; k++){
				 printf("-------\nno %i:\n", k);
				 for(l=0; l<vertex_web[k].num_neigh; l++){
				   printf("viz %i, visitado = %i\n", vertex_web[k].id_neigh[l], vertex_web[k].visited[l]);
				 }
				}*/
				/*
				if(j==0){
				  printf("Attention! All of Vertex %i neighbours are already visited. There might be a problem with the edge atribution\n",lista_viz[i]);
				}*/
				
			  }
			  
			}
			
//			printf("viz_escolhido: %i\n", viz_escolhido);						
			
			
			/* TIRAR o index 'j' para o inicio */
			for(j=0; j<vertex_web[prox_no].num_neigh; j++){
			  if(vertex_web[prox_no].id_neigh[j] == viz_escolhido && vertex_web[prox_no].visited[j]==false){
				break;
			  }
			}
			
			no_ant = prox_no;
			prox_no = viz_escolhido;
			
		  }
		  
		  
		}
		
		if(sai){ //acrescentar o regresso ao desvio:
		  
//		  printf("acrescentar o regresso ao desvio\n");
		  for(i=i_reg-1; i>=0; i--){			 
			desvio[i_desvio] =  regresso[i];
			i_desvio++;
		  }
		  
		  //desvio terminou e para sair do while, independentemente de termos atingido o num_arcos.		  
		  break; //sai do while		  

		}
		
	  } //else (arcos_ja_visitados < num_arcos - decidir no proximo no)
	  	  

//	 for(i=0; i<i_desvio; i++){
//	   printf("desvio[%i] = %i\n", i, desvio[i]);
//	 }
	 
	 /*ADICIONAR DESVIO AO CAMINHO DE IDA AQUI (actualizar num. elementos) */

	  for(i=1; i<i_desvio; i++){
//		printf("desvio[%i] = %i\n", i, desvio[i]);
		caminho_de_ida[elem_c_ida] = desvio[i];
//		printf("caminho de ida [%i] = %i\n", elem_c_ida, caminho_de_ida[elem_c_ida]);
		elem_c_ida++;
	  }
	   	   
	 }	//while (estamos no desvio)
	 

	 
  }//while (estamos no CP)
  
  //mostrar_estado_nos(num_nos_total);
  
  sai = check_visited (vertex_web, dimension);
  return sai;
  
}

int computar_custo_caminho_final (vertex *vertex_web, int *caminho_final, int elem_caminho_final){
  
  int i,j;
  int custo_final = 0;
  int anterior, proximo;
  
  for(i=1; i<elem_caminho_final; i++){
	
	anterior = caminho_final[i-1];
	proximo = caminho_final[i];
	
	for(j=0; j<vertex_web[anterior].num_neigh; j++){
	 
	  if(vertex_web[anterior].id_neigh[j] == proximo){
		custo_final += vertex_web[anterior].cost[j];
		break;
	  }
	  
	}
	
  }
  
  return custo_final;
  
}

int cyclic (uint dimension, vertex *vertex_web, int *caminho_final) {
  
  int caminho_principal [dimension+1]; //pode ser apenas parcial como pode ser circuito de hamilton
  int elem_caminho;
  int custo_caminho;
  
  bool testa = false;
  bool hamilton_path = false;
  bool cycle = false;
  int i;
  uint j;
  
  clear_visited(vertex_web,dimension);

  for(i=0; i<dimension; i++){
	
	testa = UHC ( vertex_web, vertex_web[i].id, caminho_principal, dimension);
	  
	  if(testa){
		
		//verificar caminho_principal:
/*		 for(i=0; i<dimension; i++){
			printf("caminho_principal[%i] = %i\n", i, caminho_principal[i]); 
		  } 
*/
		int hcycle;
		
		printf("Hamilton Path found.\n");
		hamilton_path = true;
		
		//verificar se e d facto um caminho ou um circuito (1º elemento e ultimo sao vizinhos?):
		hcycle = is_neigh ( caminho_principal[0], caminho_principal[dimension-1], vertex_web, dimension);			
		
		//e um ciclo de hamilton - podes sair:
		if(hcycle>-1){
		  printf("Hamilton circuit found.\n");
		  caminho_principal [dimension] = caminho_principal [0]; //repetir o primeiro.
		  cycle = true;
		  break;
		}
		
	  } 
  }
  
  // nesta fase: se hamilton_path e false - longest path... 
  // se for true - ver s hamilton_cycle tb e true...

  if(!hamilton_path){
	
	printf("No Hamilton Circuit/Path found.\n\n");
	
	int *ciclo_principal = new int[dimension]; 
	int elem_ciclo = 0, custo_ciclo = 0;
	int seed = 1000;

	//PROCURAR CICLO:
	procurar_ciclo(vertex_web, dimension, ciclo_principal, elem_ciclo, custo_ciclo, seed);
  	
	clear_visited(vertex_web, dimension);		
	
	//LONGEST PATH APARTIR DOS VIZINHOS UNICOS:
	caminho_apartir_vizinhos_unicos(vertex_web, dimension, caminho_principal, elem_caminho, custo_caminho, seed);
	
	
	/* NESTA FASE: DECIDIR ENTRE CICLO E LONGEST PATH [VERIFICAR QUAL O MELHOR METODO DE DECISAO]: */
	/* caso haja CICLO */
	if(elem_ciclo>=(dimension/2)){ //caminho principal passa a ser o ciclo.
	  
	  custo_caminho = custo_ciclo;
	  elem_caminho=elem_ciclo;
	  
	  for(i=0; i<elem_caminho; i++){
		caminho_principal [i] = ciclo_principal[i];
	  }
	  
	  cycle = true;	  
  
	}//nao precisa de else - caminho principal ja e o longest path.
	
	
	delete [] ciclo_principal;
	
	
/*	if(cycle){printf("\nMain path (cycle): ");}else{printf("Main path (longest path): ");}
	for(i=0; i<elem_caminho; i++){
	  if(i==elem_caminho-1){printf("%i\n", caminho_principal[i]);}else{printf("%i, ", caminho_principal[i]);}
	}
	printf("Number of elements = %i\nCost = %i\n", elem_caminho, custo_caminho);
*/	
	
  }else{
	
	if(cycle){ //HAMILTON CIRCUIT (CYCLE)
	
	  //ult.elemento = primeiro:
	  elem_caminho = dimension+1; 
	  
	}else{ //HAMILTON PATH
	  
	   elem_caminho = dimension; 
	 }
	
	//verificar caminho_principal:
	printf("Hamilton Path/Circuit: ");	
	custo_caminho=0;
	
	for(i=0; i<elem_caminho; i++){
	  
	  if(i==elem_caminho-1){
		printf("%i\n", caminho_principal[i]);
		printf("cost = %i\n", custo_caminho);
		
	  }else{
		printf("%i, ", caminho_principal[i]);
		
		/*COMPUTAR O CUSTO DO CAMINHO DE HAMILTON: */
		for(j=0; j<vertex_web[ caminho_principal[i] ].num_neigh; j++){			  
		  if( vertex_web[ caminho_principal[i] ].id_neigh[j] == caminho_principal[i+1] ){				
			custo_caminho += vertex_web[ caminho_principal[i] ].cost[j];
			break;
		  }				
		}	
		
	  }
	}	
  }
  
  int *caminho_de_ida = new int[3*dimension];  //e suficiente?
  int elem_c_ida;
  
  if (!hamilton_path){
  
   /* Já temos caminho principal...agora desvios... */
   
  int num_max = elem_caminho;
  int num_min = 1;
  bool resultado = false, sai = false;
  double tentativa = 1;
  int num_arcos = num_min;
  
  //garantir que num_min e minorante:
  clear_visited(vertex_web, dimension); 
  elem_c_ida = 0;
  resultado = computar_caminho_de_ida (vertex_web, dimension, caminho_de_ida, elem_c_ida, caminho_principal, elem_caminho, num_min);
//  printf("Minorante testado\n");
  if(resultado == true){ /*printf("num_arcos e 1!! Sai logo!!\n");*/ sai=true; } //se der so com 1 arco -> ja encontramos o num_arcos!
//  printf("\n");
  
  resultado = false;
  
  int prob_reg = 0;
  bool haprob = false;
  
	if(!sai){
	  while (!resultado){ //garantir que num_max e majorante.
		
		clear_visited(vertex_web, dimension); 
		elem_c_ida = 0;
		resultado = computar_caminho_de_ida (vertex_web, dimension, caminho_de_ida, elem_c_ida, caminho_principal, elem_caminho, num_max);
//		printf("Majorante testado\n");
//		printf("num_max = %i\n\n", num_max);
		
		tentativa+=0.5;
		
		if(!resultado) {
		  num_min = num_max;
		  num_max = tentativa*elem_caminho;
		  prob_reg++;
		}
		
		//MECANISMO DE PROTECCAO:
		if(prob_reg == 100){
		  resultado = true;
		  num_max = elem_caminho;
		  haprob = true;
		}
		
	  } 

	num_arcos = (num_min + num_max)/2;  
	if ( (num_min + num_max)%2 == 1 ){num_arcos++;}

	tentativa = 0;
  
  }
  
  while(sai==false && haprob==false){
	
	tentativa++;
	
	clear_visited(vertex_web, dimension);
	elem_c_ida = 0;
	resultado = computar_caminho_de_ida (vertex_web, dimension, caminho_de_ida, elem_c_ida, caminho_principal, elem_caminho, num_arcos); 
	
	
//	printf("tentativa %i (num_arcos = %i)\n", (int)tentativa,num_arcos);
	
	if(resultado){
	  
	  num_max = num_arcos;
	  
	  if( num_max - 1 == num_min) {
		sai=true;
	  
	  }else{	  
		num_arcos = (num_min + num_max)/2;	
		if ( (num_min + num_max)%2 == 1 ){num_arcos++;}
	  }	  
	  
	}else{
	  
	  num_min = num_arcos;
	  
	  if( num_max - 1 == num_min) {
		num_arcos++;
	  
	  }else{  
		
		num_arcos = (num_min + num_max)/2;
		if ( (num_min + num_max)%2 == 1 ){num_arcos++;}
		
	  }
	  
	}
	
  }
  
  /* nesta fase tenho caminho_de_ida computado. */

/* printf("caminho de ida:\n");
 for(i=0; i<elem_c_ida; i++){
	if(i==elem_c_ida-1){ printf("%i\n", caminho_de_ida[i]); }else{ printf("%i, ", caminho_de_ida[i]); }
 }
 printf("elementos_c_ida = %i\n", elem_c_ida);   */


  }else{	//hamilton cycle or path
	  //caminho_de_ida = caminho_principal
	  	  
	  for (i=0; i<elem_caminho; i++){
		caminho_de_ida [i] = caminho_principal[i];
	  }
	  elem_c_ida = elem_caminho;
  }
  
  
  /* computar custo do caminho de ida (+ volta) */
  
  int elem_caminho_final = 2*elem_c_ida-1;
  //int caminho_final [elem_caminho_final];
  
  //1ª parte do caminho final e igual ao caminho de ida para todos os casos:
  for(i=0; i<elem_c_ida; i++){
	caminho_final [i] = caminho_de_ida[i];			  
  }


  
  if(!cycle){ //add inverse_path to (Hamilton/Longest) Path
    
	  for(i=elem_c_ida-2; i>=0; i--){
		caminho_final [2*(elem_c_ida-1) - i] = caminho_de_ida[i];
	  }
	  
  }else{ //ciclico: caminho de ida = caminho final.		
	elem_caminho_final = elem_c_ida;		  
  }

  
  delete [] caminho_de_ida; 
  
  
  /*printf("Final Path: ");
  for(i=0; i<elem_caminho_final; i++){
	if(i==elem_caminho_final-1){ printf("%i\n", caminho_final[i]); }else{ printf("%i, ", caminho_final[i]); }
  }
  printf("Number of elements = %i\n", elem_caminho_final);*/
  
  i = computar_custo_caminho_final (vertex_web, caminho_final, elem_caminho_final);
  //printf("Cost = %i\n\n", i);
  
  //mecanismo contra loops infinitos:
  //if(haprob){ printf("Warning: Couldn't access all nodes.\n"); }
  
  clear_visited(vertex_web, dimension);    
  
  return elem_caminho_final;
  
} //fim da função

void shift_cyclic_path (uint start_vertex, int *caminho_final, int elem_caminho_final){
  
  uint posshift = 0;
  int i;
  int temp_table[elem_caminho_final];
  
  for (i=0; i<elem_caminho_final; i++){
    if (caminho_final[i] == start_vertex){
     posshift = i;
     break;
    }
  }
  
  uint aux;
  i=0;
  
  for(aux=posshift; aux<elem_caminho_final-1; aux++){
   temp_table[i] = caminho_final[aux];
   i++;
  }
  
  for(aux=0; aux<=posshift; aux++){
   temp_table[i]=caminho_final[aux];
   i++;
  }
  
  for(i=0; i<elem_caminho_final;i++){
   caminho_final[i] = temp_table[i]; 
  }
  
}

uint get_MSP_dimension (char* msp_file) {
	
	FILE *file;
	file = fopen (msp_file,"r");
	uint dimension;
	
	if(file == NULL){
		ROS_INFO("Can not open filename %s", msp_file);
		ROS_BREAK();	
	}else{
		ROS_INFO("MSP Route File Opened. Reading Dimensions.\n");
		fscanf (file, "%u", &dimension);
	}
	fclose(file);
	return dimension;
}

void get_MSP_route (uint *route, uint dimension, char* msp_file) {
	
   FILE *file;
   file = fopen (msp_file,"r");
   
   if(file == NULL){
      ROS_INFO("Can not open filename %s", msp_file);
      ROS_BREAK();	
   }else{
      ROS_INFO("MSP Route File Opened. Getting MSP Route.\n");
      
      uint i;
      float temp;
      
      //Start Reading the File from the second line On:
      //Start Reading the File from FIRST_VID On:
	fscanf (file, "%f", &temp);   

      
      for (i=0;i<dimension;i++){	
	fscanf (file, "%u", &route[i]);	
      }     
      
   }
	
    //printf ("[v=10], x = %f (meters)\n",vertex_web[10].x); 

   fclose(file);
}
