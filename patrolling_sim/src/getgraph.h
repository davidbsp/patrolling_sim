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

#include <stdio.h>
#include <stdlib.h>
#include <cmath>

//File Line of the First Vertex ID to read (Protection) - fscanf() ignores blank lines
#define FIRST_VID 5

typedef unsigned int uint;

typedef struct {
  uint id, num_neigh;
  float x, y; 		//pass these attributes in meters
  uint id_neigh[8], cost[8];
  bool visited[8];
  char dir [8][3];	//table of 8 strings with 3 chars max ("N","NE","E","SE","S","SW","W","NW")
}vertex;

uint WIDTH_PX;
uint HEIGHT_PX;
float RESOLUTION;
float WIDTH_M;
float HEIGHT_M;

uint GetGraphDimension (char* graph_file){
  
   FILE *file;
   file = fopen (graph_file,"r");
   uint dimension;
   
   if(file == NULL){
      ROS_INFO("Can not open filename %s", graph_file);
      ROS_BREAK();	
   }else{
      ROS_INFO("Graph File Opened. Reading Dimensions.\n");
      fscanf (file, "%u", &dimension);
      
      //Initialize other dimension variables:
      fscanf (file, "%u", &WIDTH_PX);
      fscanf (file, "%u", &HEIGHT_PX);
      fscanf (file, "%f", &RESOLUTION);
      WIDTH_M = (float) WIDTH_PX * RESOLUTION;
      HEIGHT_M = (float) HEIGHT_PX * RESOLUTION;
   }
   fclose(file);
   return dimension;
}


void GetGraphInfo (vertex *vertex_web, uint dimension, char* graph_file){
   
   FILE *file;
   file = fopen (graph_file,"r");
   
   if(file == NULL){
      ROS_INFO("Can not open filename %s", graph_file);
      ROS_BREAK();	
   }else{
      ROS_INFO("Graph File Opened. Getting Graph Info.\n");
      
      uint i,j;
      float temp;
      
      //Start Reading the File from FIRST_VID On:
      for (i=0; i<FIRST_VID-1; i++){
	 fscanf (file, "%f", &temp);
      }      

      for (i=0;i<dimension;i++){
	
	fscanf (file, "%u", &vertex_web[i].id);
	
	fscanf (file, "%f", &vertex_web[i].x);
	vertex_web[i].x *= RESOLUTION; //convert to m	
	
	fscanf (file, "%f", &vertex_web[i].y);
	vertex_web[i].y *= RESOLUTION; //convert to m
	
	fscanf (file, "%u", &vertex_web[i].num_neigh);
	
	for (j=0;j<vertex_web[i].num_neigh; j++){
	  fscanf (file, "%u", &vertex_web[i].id_neigh[j]);
	  fscanf (file, "%s", &vertex_web[i].dir[j]);
	  fscanf (file, "%u", &vertex_web[i].cost[j]);	//can eventually be converted to meters also...
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