#!/usr/bin/env python
#

import Tkinter as tk
import tkMessageBox
import ConfigParser
import thread
from Tkinter import *
from ttk import *
import Image, tkFileDialog
import numpy as np
import sys, time, os, glob, shutil
from math import atan2, degrees, radians

import datetime

Alg_names = [ 
        [ 'RAND', 'Random' ],
        [ 'CR',   'Conscientious_Reactive' ],
        [ 'HCR',  'Heuristic_Conscientious_Reactive' ],
        [ 'HPCC', 'Conscientious_Cognitive' ],
        [ 'CGG',  'Cyclic' ],
        [ 'MSP',  'MSP' ],
        [ 'GBS',  'GBS' ],
        [ 'SEBS', 'SEBS' ],
        [ 'DTAG', 'DTAGreedy' ],
        [ 'DTAS', 'DTASSI' ],
        [ 'DTAP', 'DTASSIPart' ]
     ]

Map_names = ['cumberland','example','grid','1r5','DISlabs']   

NRobots_list = ['1','2','4','6','8','12']

LocalizationMode_list = ['odom','GPS']

Terminal_list = ['gnome-terminal','xterm']

initPoses = {}

# return long name of the algorithm
def findAlgName(alg):
    r = 'None'
    for i in range(0,len(Alg_names)):
        if (Alg_names[i][0]==alg):
            r = Alg_names[i][1]
    return r

# load initial poses from configuration file
def loadInitPoses():
  try:
    ConfigIP = ConfigParser.ConfigParser()
    ConfigIP.read("params/initial_poses.txt")
    for option in ConfigIP.options("InitialPoses"):
        #print option
        initPoses[option] = ConfigIP.get("InitialPoses", option)
  except:
    print "Could not load initial poses file"


# get ROS time from /clock topic
def getROStime():
    os.system("rostopic echo -n 1 /clock > rostime.txt")
    f = open('rostime.txt','r')
    t = 0
    for line in f:
        if (line[2:6]=='secs'):
            t = int(line[8:])
    f.close()
    return t

# get running simulation flag from /simulation_runnning param
def getSimulationRunning():
    os.system("rosparam get /simulation_runnning > simrun.txt")
    f = open('simrun.txt','r')
    t = True
    line = f.readline();
    if (line[0:5]=='false'):
        t = False
    f.close()
    return t

# Run the experiment with the given arguments
# Terminates if simulation is stopped (/simulation_runnning param is false)
# or if timeout is reached (if this is >0)
def run_experiment(MAP, NROBOTS, ALG_SHORT, LOC_MODE, TERM, TIMEOUT):
    ALG = findAlgName(ALG_SHORT)
    print 'Run the experiment'
    print 'Loading map ',MAP
    print 'N. robot ',NROBOTS
    print 'Algorithm ',ALG,'  ',ALG_SHORT
    print 'Localization Mode ',LOC_MODE
    print 'Terminal ',TERM
    print 'Timeout ',TIMEOUT

    loadInitPoses()

    scenario = MAP+"_"+NROBOTS
    iposes = initPoses[scenario.lower()]
    print scenario,'   ',iposes
    
    if (TERM == 'xterm'):
        os.system('xterm -e roscore &')
    else:
        os.system('gnome-terminal -e "bash -c \'roscore\'" &')
    os.system('sleep 3')
    os.system('rosparam set /use_sim_time true')

    cmd = './setinitposes.py '+MAP+' "'+iposes+'"'
    print cmd
    os.system(cmd)
    os.system('sleep 1')

    cmd_monitor = 'rosrun patrolling_sim monitor '+MAP+' '+ALG_SHORT+' '+NROBOTS        
    cmd_stage = 'roslaunch patrolling_sim map.launch map:='+MAP
    print cmd_monitor
    print cmd_stage
    if (TERM == 'xterm'):
        os.system('xterm -e  "'+cmd_monitor+'" &') 
        os.system('xterm -e  "'+cmd_stage+'" &')
    else:
        os.system('gnome-terminal --tab -e  "bash -c \''+cmd_monitor+'\'" --tab -e "bash -c \''+cmd_stage+'\'" &')
    
    os.system('sleep 3')
    
    # Start robots
    if (LOC_MODE == 'odom'):
        robot_launch = 'robot.launch'
    else:
        robot_launch = 'robot_fake_loc.launch'
    
    gcmd = 'gnome-terminal '
    for i in range(0,int(NROBOTS)):
        print 'Run robot ',i
        cmd = 'bash -c \'roslaunch patrolling_sim '+robot_launch+' robotname:=robot_'+str(i)+' mapname:='+MAP+'\''
        print cmd
        #os.system('xterm -e  "'+cmd+'" &')
        #os.system('sleep 1')
        gcmd = gcmd + ' --tab -e "'+cmd+'" '
    gcmd = gcmd + '&'    
    print gcmd
    os.system(gcmd)
    os.system('sleep 5')    
        
    # Start patrol behaviors
    gcmd = 'gnome-terminal '
    for i in range(0,int(NROBOTS)):
        print 'Run patrol robot ',i
        if (ALG_SHORT=='MSP'):
            cmd = 'bash -c \'rosrun patrolling_sim '+ALG+' __name:=patrol_robot'+str(i)+' '+MAP+' '+str(i)+' MSP/'+MAP+'/'+MAP+'_'+str(NROBOTS)+'_'+str(i)+' '+'\''
        elif (ALG_SHORT=='GBS' or ALG_SHORT=='SEBS'):
            cmd = 'bash -c \'rosrun patrolling_sim '+ALG+' __name:=patrol_robot'+str(i)+' '+MAP+' '+str(i)+' '+str(NROBOTS)+'\''
        else:
            now = datetime.datetime.now()
            dateString = now.strftime("%Y-%m-%d-%H:%M")
# FOR DEBUG            cmd = 'bash -c \'rosrun patrolling_sim '+ALG+' __name:=patrol_robot'+str(i)+' '+MAP+' '+str(i)+' > logs/'+ALG+'-'+dateString+'-robot'+str(i)+'.log \''
            cmd = 'bash -c \'rosrun patrolling_sim '+ALG+' __name:=patrol_robot'+str(i)+' '+MAP+' '+str(i)+'\''
        print cmd
        #os.system('xterm -e  "'+cmd+'" &')
        #os.system('sleep 1')
        gcmd = gcmd + ' --tab -e "'+cmd+'" '
    gcmd = gcmd + '&'    
    print gcmd
    os.system(gcmd)
    os.system('sleep 5')
    # wait for termination
    run = True
    while (run):
        t = getROStime()
        #print "Elapsed time: ",t," secs."
        if ((TIMEOUT>0 and t>TIMEOUT) or (not getSimulationRunning())):
            run = False;
        else:
            os.system('sleep 1')
    os.system("./stop_experiment.sh")


class DIP(tk.Frame):
    def __init__(self, parent):
        tk.Frame.__init__(self, parent) 
        self.parent = parent        
        self.initUI()
        
    def initUI(self):
        self.loadOldConfig()
        
        
        self.parent.title("MRP Experiment Launcher")
        self.style = Style()
        self.style.theme_use("alt")
        self.parent.resizable(width=FALSE, height=FALSE)
        self.pack(fill=BOTH, expand=1)
        
        #self.columnconfigure(1, weight=1)
        #self.columnconfigure(3, pad=7)
        #self.rowconfigure(3, weight=1)
        #self.rowconfigure(7, pad=7)
        
        lbl = Label(self, text="Map")
        lbl.grid(sticky=W, row = 0, column= 0, pady=4, padx=5)
                
        self.map_name_list = Map_names
        self.map_ddm = StringVar(self)
        try:
            lastmap=self.oldConfigs["map"]
        except:
            lastmap=self.map_name_list[0]
        self.map_ddm.set(lastmap)
        tk.OptionMenu(self, self.map_ddm, *self.map_name_list).grid(sticky=W, row=0, column=1, pady=4, padx=5)

        lbl = Label(self, text="N. Robots")
        lbl.grid(sticky=W, row = 1, column= 0, pady=4, padx=5)

        self.robots_n_list = NRobots_list
        self.robots_ddm = StringVar(self)
        try:
            lastnrobots=self.oldConfigs["nrobots"]
        except:
            lastnrobots=self.robots_n_list[0]
        self.robots_ddm.set(lastnrobots)
        tk.OptionMenu(self, self.robots_ddm, *self.robots_n_list).grid(sticky=W, row=1, column=1, pady=4, padx=5)

        lbl = Label(self, text="Algorithm")
        lbl.grid(sticky=W, row = 2, column= 0, pady=4, padx=5)

        self.algorithm_list = []
        for i in range(0,len(Alg_names)):
            self.algorithm_list += [Alg_names[i][0]]

        self.alg_ddm = StringVar(self)
        try:
            lastalg=self.oldConfigs["algorithm"]
        except:
            lastalg=self.algorithm_list[0]
        self.alg_ddm.set(lastalg)
        tk.OptionMenu(self, self.alg_ddm, *self.algorithm_list).grid(sticky=W, row=2, column=1, pady=4, padx=5)
        

        lbl = Label(self, text="Localization Mode")
        lbl.grid(sticky=W, row = 3, column= 0, pady=4, padx=5)

        self.locmode_list = LocalizationMode_list
        self.locmode_ddm = StringVar(self)
        try:
            lastlocmode=self.oldConfigs["locMode"]
        except:
            lastlocmode=self.locmode_list[0]
        self.locmode_ddm.set(lastlocmode)
        tk.OptionMenu(self, self.locmode_ddm, *self.locmode_list).grid(sticky=W, row=3, column=1, pady=4, padx=5)


        lbl = Label(self, text="Terminal")
        lbl.grid(sticky=W, row = 4, column= 0, pady=4, padx=5)

        self.term_list = Terminal_list
        self.term_ddm = StringVar(self)
        try:
            lastterm=self.oldConfigs["term"]
        except:
            lastterm=self.term_list[0]
        self.term_ddm.set(lastterm)
        tk.OptionMenu(self, self.term_ddm, *self.term_list).grid(sticky=W, row=4, column=1, pady=4, padx=5)
  
        launchButton = Button(self, text="Start Experiment",command=self.launch_script)
        launchButton.grid(sticky=W, row=5, column=0, pady=4, padx=5)
        
        launchButton = Button(self, text="Stop Experiment",command=self.kill_demo)
        launchButton.grid(sticky=W, row=5, column=1, pady=4, padx=5)
        
    
    def launch_script(self):
        self.saveConfigFile();
        thread.start_new_thread( run_experiment, (self.map_ddm.get(),self.robots_ddm.get(),self.alg_ddm.get(),self.locmode_ddm.get(),self.term_ddm.get(),0) )

    
    def quit(self):
      self.parent.destroy()
      
    def kill_demo(self):
      os.system("rosparam set /simulation_runnning false")
      
      
    def saveConfigFile(self):
      f = open('lastConfigUsed', 'w')
      f.write("[Config]\n")
      f.write("map: %s\n"%self.map_ddm.get())
      f.write("nrobots: %s\n"%self.robots_ddm.get())
      f.write("algorithm: %s\n"%self.alg_ddm.get())
      f.write("locmode: %s\n"%self.locmode_ddm.get())
      f.write("term: %s\n"%self.term_ddm.get())
      f.close()


    def loadOldConfig(self):
      try:
        self.oldConfigs = {}
        self.Config = ConfigParser.ConfigParser()
        self.Config.read("lastConfigUsed")
        for option in self.Config.options("Config"):
          self.oldConfigs[option] = self.Config.get("Config", option)
      except:
        print "Could not load config file"


    


def main():

  if (len(sys.argv)==1):
    root = tk.Tk()
    DIP(root)
    root.geometry("300x240+0+0")
    root.mainloop()  
  elif (len(sys.argv)==7):
    MAP = sys.argv[1]
    NROBOTS = sys.argv[2]
    ALG_SHORT = sys.argv[3]
    LOC_MODE = sys.argv[4]
    TERM = sys.argv[5]
    TIMEOUT = int(sys.argv[6])
    run_experiment(MAP, NROBOTS, ALG_SHORT, LOC_MODE, TERM, TIMEOUT)
  else:
    print "Use: ",sys.argv[0]
    print " or  ",sys.argv[0],' <map> <n.robots> <alg_short> <loc_mode> <term> <timeout>'
  



if __name__ == '__main__':
    main()

