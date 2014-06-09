#!/bin/bash
# Maps: grid, example, cumberland, DISlabs
# Alg: RAND,CR,HCR,HPCC,CGG,MSP,GBS,SEBS,DTAG,DTAS
# Loc mode: odom, GPS
# Term: gnome-terminal,xterm

MAP=DISlabs
NROBOTS=4
ALG=DTAS
LOC=odom
TERM=gnome-terminal 
TIMEOUT=1800

./start_experiment.py $MAP $NROBOTS $ALG $LOC $TERM $TIMEOUT
