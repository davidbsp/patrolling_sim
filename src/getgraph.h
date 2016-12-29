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
* Author: David Portugal (2011-2014), and Luca Iocchi (2014-2016)
*********************************************************************/

#ifndef __GETGRAPH_H__
#define __GETGRAPH_H__

#include <stdio.h>
#include <stdlib.h>
#include <cmath>

//File Line of the First Vertex ID to read (Protection) - fscanf() ignores blank lines
#define FIRST_VID 7

typedef unsigned int uint;

typedef struct {
  uint id, num_neigh;
  float x, y; 		//pass these attributes in meters
  uint id_neigh[8], cost[8];
  float cost_m[8];
  bool visited[8];
  char dir [8][3];	//table of 8 strings with 3 chars max ("N","NE","E","SE","S","SW","W","NW")
}vertex;

extern uint WIDTH_PX;
extern uint HEIGHT_PX;
extern float RESOLUTION;
extern float WIDTH_M;
extern float HEIGHT_M;

uint GetGraphDimension (const char* graph_file);

void GetGraphInfo (vertex *vertex_web, uint dimension, const char* graph_file);
  
uint IdentifyVertex (vertex *vertex_web, uint size, double x, double y);

uint GetNumberEdges (vertex *vertex_web, uint dimension);
  
//integer to array (itoa for linux c)
char* itoa(int value, char* str, int radix);


#endif
