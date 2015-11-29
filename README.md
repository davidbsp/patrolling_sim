===============================
patrolling_sim v2.2 (Nov. 2015)
===============================

patrolling_sim for ROS (Groovy/Hydro/Indigo) -- catkin version

Authors:

Main framework and basic algorithms:
 David Portugal (2011-2014), Luca Iocchi (2014)
 
Additional algorithms:
* DTAP: Alessandro Farinelli (2014)


*** NOTE ***
Fork https://github.com/gennari/patrolling_sim implements a distributed execution
of the patrolling environment that can be used also on real robots. 
These two branches will be merged soon.
************

This package contains the implementation of several algorithms for multi-robot patrolling and a general structure of a PatrolAgent that can be extended to implement other ones.
It extends previous version of patrolling_sim with an improved structure of the code that allows easy integration of new algorithms, an improved navigation configuration that allows the robots to move at 1 m/s and to avoid most of conflicting situations, and a better management of the experiments and generation of the results.

For a quick try, just compile the workspace ('catkin_make'), start the script './start_experiment.py',
make your choices and see the experiment running.

WARNING: sometimes (on some machines) the very first run does not work, because of timing problems with roscore. 
Either restart the experiment a second time, or run roscore once before starting the experiment.

Several maps are available in the 'maps' folder. For map X the patrol graph is visible in the file
maps/X/X-graph.png 

Several algorithms have been implemented in the 'src' folder. Each method is implemented through a class 'X_Agent' that inherits from the abstract class 'PatrolAgent' many common services and functions.

Results of the experiments are stored in the 'results' folder.

In order to run a particular experiment or a set of experiments, use the run-exp.sh script template.
It is convenient to copy this file in a new file that you can edit as you wish.
For example, the current version of run_exp.sh allows to run an experiment for 
DISlabs, with 8 robots, 30 minutes, using DTAP algorithm, and other standard parameters.
After 30 minutes the experiment terminates and the results will be available in the files
result/{map}_{n.robots}/{algorithm}/{machine}/{date}*.csv

The info file of an experiment contains a summary of the results of the experiments with the following values:
Map ; N. robots ; Wait time	; Communication delay ;	Algorithm ;	Algorithm parameters ; Machine ; Date ; Time ; Real time ; Interferences ; Termination ; Idleness	min ;	avg	; stddev ; max

The script can be extended to run multiple experiments in a single session, by just adding new commands like the one in the examples (possibly with different parameters).

*** NEW NAVIGATION MODULES ***

Default navigation modules are standard ROS nodes amcl (localization) and move_base (path planning and motion control).
They work (usually) fine, but they require many computational resources that may limit the simulation of many robots 
in a single machine.

New navigation modules, called thin_navigation, are available at https://bitbucket.org/ggrisetti/thin_navigation.
thin_localizer and thin_planner have the same interface as amcl and move_base so they can be used in their replacement
by just changing the launch file. To install thin_navigation, just download the package in a catkin workspace and
compile it with catkin_make (see README file for other details).

The thin_navigation modules have been fully integrated in patrolling_sim. Just select thin_navigation instead of ros
as navigation module either in the start_experiment.py GUI or in the run_exp.sh script.

Warning: the thin_navigation modules are still under testing and debug!!!


*** NEW SUPPORT FOR EXTENDED STAGE API ***

NOTE: Extended API for stage are available with customized versions of stage and stageros.
(See https://github.com/iocchi/stage_ros for details).

With the use of the extended API it is possible to control activation of some GUI elements
in Stage (e.g., footprints, simulation speedup, and screenshots).
Set the Custom Stage flag to true in run_exp.sh script to activate these functions.


