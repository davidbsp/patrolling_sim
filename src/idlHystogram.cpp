#include <stdlib.h>
#include <math.h>
#include <vector>
#include <fstream>
#include <iostream>
#include <cstring>

using namespace std;

#define RESOLUTION 5.0 // seconds

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
    
    double time,idleness; int robot,node; char ch;
    int sum=0;
    while (f.good()) {
        // current_time,id_robot,goal,current_idleness[goal]);
        f >> time; f>>ch;
        //f >> robot; f>>ch;
        f >> node; f>>ch;
        f >> idleness; f>>ch;
        
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
    
    char nf[strlen(argv[1])+8];
    sprintf(nf,"%s.hyst",argv[1]);
    cout << "Hystogram output file: " << nf << endl;
    ofstream of; of.open(nf);
    for (int k=0; k<n; k++) {
        of << (double)v[k]/sum << endl;
    }
    
    of.close();
    
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


