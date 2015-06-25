==========================
patrolling_sim v2.1 (2015)
==========================

patrolling_sim for ROS (Groovy/Hydro/Indigo) - ROSbuild version

Authors:

Main framework and basic algorithms:
 David Portugal (2011-2014), Luca Iocchi (2014)
 
Additional algorithms:
* DTAS, DTAP: Alessandro Farinelli (2014)


*** NOTE ***
Fork https://github.com/gennari/patrolling_sim implements a distributed execution
of the patrolling environment that can be used also on real robots. 
These two branches will be merged soon.
************


This package contains the implementation of several algorithms for multi-robot patrolling
and a general structure of a PatrolAgent that can be extended to implement other ones.

For a quick try, just compile the package ('rosmake'), start the script './start_experiment.py',
make your choices and see the experiment running.
WARNING: sometimes (on some machines) the very first run does not work, because of timing problems with roscore. 
Either restart the experiment a second time, or run roscore once before using starting the experiment.

Several maps are available in the 'maps' folder. For map X the patrol graph is visible in the file
patrolling_sim/maps/X/X-graph.png 

Several algorithm have been implemented in the 'src' folder. 
Each method is implemented through a class 'X_Agent'
that inherits from the abstract class 'PatrolAgent' many common services and functions.

Results of the experiments are stored in the 'results' folder.

For running a particular experiment, use the run-exp.sh script template.
It is convenient to copy this file in a new file that you can edit as you wish.
For example, the current version of run_exp.sh allows to run an experiment for 
DISlabs, with 8 robots, 30 minutes, using DTAP algorithm, and other standard parameters.
After 30 minutes the experiment terminates and the results will be available in the files
result/{map}_{n.robots}/{algorithm}/{machine}/{date}*.csv

The info file contains a summary of the results of the experiments with the following values:
Map ;	N. robots ;	Wait time	; Communication delay ;	Algorithm ;	Algorithm parameters ;	Machine ;	Date ;	Time	Interferences	; Termination ;	Idleness	min ;	avg	; stddev	; max	; avg + stddev	; avg + 2 stddev	; Interf/min

											
The script can be extended to run multiple experiments in a single session, bu just adding new commands like the one in the examples (possibly with different parameters).
