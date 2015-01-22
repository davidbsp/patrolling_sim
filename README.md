patrolling_sim
==============

patrolling_sim for ROS (Groovy/Hydro/Indigo) -- catkinized version (catkin branch)

Authors:

Main framework and basic algorithms:
 David Portugal (2011-2014), Luca Iocchi (2014)
 
Additional algorithms:
* DTAS, DTAP: Alessandro Farinelli (2014)


This package contains the implementation of several algorithms for multi-robot patrolling
and a general structure of a PatrolAgent that can be extended to implement other ones.


For a quick try, just compile the workspace ('catkin_make'), start the script './start_experiment.py',
make your choices and see the experiment running.

Several maps are available in the 'maps' folder. For map X the patrol graph is visible in the file
patrolling_sim/maps/X/X-graph.png 

Several algorithm have been implemented in the 'src' folder. 
Each method is implemented through a class 'X_Agent'
that inherits from the abstract class 'PatrolAgent' many common services and functions.

Results of the experiments are stored in the 'results' folder.

In order to run a particular experiment, use the run-exp.sh script template.
It is convenient to copy this file in a new file that you can edit as you wish.
For example, the current version of run_exp.sh allows to run an experiment for 
DISlabs, with 8 robots, 30 minutes, using DTAP algorithm, and other standard parameters.
After 30 minutes the experiment terminate and the results will be available in the files
result/<map>_<n.robots>/<algorithm>/<machine>/<date>*.csv

Ths info file contains a summary of the result of the experiments with the following values:
Map ;	N. robots ;	Wait time	; Communication delay ;	Algorithm ;	Algorithm parameters ;	Machine ;	Date ;	Time	Interferences	; Termination ;	Idleness	min ;	avg	; stddev	; max	; avg + stddev	; avg + 2 stddev	; Interf/min

											
