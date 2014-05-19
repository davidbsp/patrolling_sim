#include <ros/ros.h>

#include "getgraph.h"

uint WIDTH_PX;
uint HEIGHT_PX;
float RESOLUTION;
float WIDTH_M;
float HEIGHT_M;

uint GetGraphDimension (const char* graph_file){
  
   FILE *file;
   file = fopen (graph_file,"r");
   uint dimension;
   
   if(file == NULL){
      ROS_INFO("Can not open filename %s", graph_file);
      ROS_BREAK();	
   }else{
      ROS_INFO("Graph File Opened. Reading Dimensions.\n");
      int r;
      r=fscanf (file, "%u", &dimension);
      
      //Initialize other dimension variables:
      r=fscanf (file, "%u", &WIDTH_PX);
      r=fscanf (file, "%u", &HEIGHT_PX);
      r=fscanf (file, "%f", &RESOLUTION);
      WIDTH_M = (float) WIDTH_PX * RESOLUTION;
      HEIGHT_M = (float) HEIGHT_PX * RESOLUTION;
   }
   fclose(file);
   return dimension;
}


void GetGraphInfo (vertex *vertex_web, uint dimension, const char* graph_file){
   
   FILE *file;
   file = fopen (graph_file,"r");
   
   if(file == NULL){
      ROS_INFO("Can not open filename %s", graph_file);
      ROS_BREAK();	
   }else{
      ROS_INFO("Graph File Opened. Getting Graph Info.\n");
      
      uint i,j;
      float temp;
      int r;
      
      //Start Reading the File from FIRST_VID On:
      for (i=0; i<FIRST_VID-1; i++){
        r=fscanf (file, "%f", &temp);
      }      

      for (i=0;i<dimension;i++){
	
	r=fscanf (file, "%u", &vertex_web[i].id);
	
	r=fscanf (file, "%f", &vertex_web[i].x);
	vertex_web[i].x *= RESOLUTION; //convert to m	
	
	r=fscanf (file, "%f", &vertex_web[i].y);
	vertex_web[i].y *= RESOLUTION; //convert to m
	
	r=fscanf (file, "%u", &vertex_web[i].num_neigh);
	
	for (j=0;j<vertex_web[i].num_neigh; j++){
	  r=fscanf (file, "%u", &vertex_web[i].id_neigh[j]);
	  r=fscanf (file, "%s", &vertex_web[i].dir[j]);
	  r=fscanf (file, "%u", &vertex_web[i].cost[j]);	//can eventually be converted to meters also...
	}
	
      }     
      
   }
	
    //printf ("[v=10], x = %f (meters)\n",vertex_web[10].x); 

   fclose(file);   
   	  
  }
  
uint IdentifyVertex (vertex *vertex_web, uint size, double x, double y){
  
  uint i, v=0;
  double dif_x, dif_y, result=INFINITY;
  
  for (i=0; i<size; i++){
    dif_x = vertex_web[i].x - x;
    dif_y = vertex_web[i].y - y;
    
//     printf("[%u] result = %f, (dif_x+dif_y) = %f\n",i,result, fabs(dif_x) + fabs(dif_y));
    if( result > fabs (dif_x) + fabs (dif_y) ){ //Identify the Vertex closer to the initial coordinates x & y
      result = fabs (dif_x) + fabs (dif_y);
      v = i;
    }
  }
  return v;  
}

//integer to array (itoa for linux c)
char* itoa(int value, char* str, int radix) {
    static char dig[] =
        "0123456789"
        "abcdefghijklmnopqrstuvwxyz";
    int n = 0, neg = 0;
    unsigned int v;
    char* p, *q;
    char c;

    if (radix == 10 && value < 0) {
        value = -value;
        neg = 1;
    }
    v = value;
    do {
        str[n++] = dig[v%radix];
        v /= radix;
    } while (v);
    if (neg)
        str[n++] = '-';
    str[n] = '\0';

    for (p = str, q = p + (n-1); p < q; ++p, --q)
        c = *p, *p = *q, *q = c;
    return str;
}


