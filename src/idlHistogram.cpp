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

#include <stdlib.h>
#include <math.h>
#include <vector>
#include <fstream>
#include <sstream>
#include <iostream>
#include <cstring>

using namespace std;

#define RESOLUTION 1.0 // seconds

#define MAXIDLENESS 500.0 // seconds

int main (int argc, char **argv)
{   
    if (argc<2) {
        cout << "Use: " << argv[0] << " <filename> " << endl;
        exit(-1);
    }
    
    int n = (int)(MAXIDLENESS/RESOLUTION)+1;
    int v[n]; for (int k=0; k<n; k++) v[k]=0;
    
    ifstream f;
    f.open(argv[1]);
    
    double time,idleness; int robot,node,interferences; char ch;
    int sum=0;
    while (f.good()) {
        char line[80];
        f.getline(line,80);
        stringstream ss(line);
        // current_time, id_robot, goal, current_idleness[goal], interferences
        ss >> time; ss>>ch;
        ss >> robot; ss>>ch;
        ss >> node; ss>>ch;
        ss >> idleness; ss>>ch;
        ss >> interferences;
        
        // cout << idleness << endl;
        
        if (idleness>MAXIDLENESS) {
           cout << "WARNING: Idleness " << idleness << " out of range! Discarded!" << endl; 
        }
        else {
            int b = (int)(idleness/RESOLUTION);
            v[b]++; sum++;
        }
    }
    f.close();
    
    char nf[strlen(argv[1])+8], cnf[strlen(argv[1])+8];
    strcpy(nf,argv[1]);
    nf[strlen(nf)-4]='\0';
    strcpy(cnf,nf);
    strcat(nf,".hist");
    strcat(cnf,".chist");
    cout << "Histogram output file: " << nf << endl;
    ofstream of1; of1.open(nf);
    ofstream of2; of2.open(cnf);
    double c=0;
    for (int k=0; k<n; k++) {
        of1 << k*RESOLUTION << " " << (double)v[k]/sum << endl;
        c += (double)v[k]/sum;
        of2 << k*RESOLUTION << " " << c << endl;
    }
    
    of1.close();   of2.close();
    
}

/*
 * 
 * 
 * 
    gnuplot> plot ("results/idl_grid_4_RAND.csv.hyst") w li
    gnuplot> replot ("results/idl_grid_4_DTAG.csv_OLD.hyst") w li

 * 
 * 
 * 
 * 
 */


