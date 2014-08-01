patrolling_sim
==============

patrolling_sim for ROS Groovy

Authors:

Main framework and basic algorithms:
 David Portugal (2011-2014), Luca Iocchi (2014)
 
Additional algorithms:
* DTAS, DTAP: Alessandro Farinelli (2014)


This package contains the implementation of several algorithms for multi-robot patrolling
and a general structure of a PatrolAgent that can be extended to implement other ones.


For a quick try, just compile the package ('rosmake'), start the script './start_experiment.py',
make your choices and see the experiment running.

Several maps are available in the 'maps' folder. For map X the patrol graph is visible in the file
patrolling_sim/maps/X/X-graph.png 

Several algorithm have been implemented in the 'src' folder. 
Each method is implemented through a class 'X_Agent'
that inherits from the abstract class 'PatrolAgent' many common services and functions.

Results of the experiments are stored in the 'results' folder.
