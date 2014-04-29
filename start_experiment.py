#!/usr/bin/env python
#

import Tkinter as tk
import tkMessageBox
import ConfigParser
from Tkinter import *
from ttk import *
import Image, tkFileDialog
import numpy as np
import sys, time, os, glob, shutil
from math import atan2, degrees, radians

Alg_names = [ 
        [ 'CR',   'Conscientious_Reactive2' ],
        [ 'HCR',  'Heuristic_Conscientious_Reactive2' ],
        [ 'HPCC', 'Conscientious_Cognitive2' ],
        [ 'CGG',  'Cyclic2' ],
        [ 'MSP',  'MSP2' ],
        [ 'GBS',  'GBS2' ],
        [ 'SEBS', 'SEBS2' ]
     ]

Map_names = ['cumberland','example','grid','1r5']   

NRobots_list = ['1','2','4','6','8','12']


# return long name of the algorithm
def findAlgName(alg):
    r = 'None'
    for i in range(0,len(Alg_names)):
        if (Alg_names[i][0]==alg):
            r = Alg_names[i][1]
    return r


class DIP(tk.Frame):
    def __init__(self, parent):
        tk.Frame.__init__(self, parent) 
        self.parent = parent        
        self.initUI()

    def centerWindow(self):
        w = 400
        h = 200
        sw = self.parent.winfo_screenwidth()
        sh = self.parent.winfo_screenheight()
        x = (sw - w)/2
        y = (sh - h)/2
        self.parent.geometry('%dx%d+%d+%d' % (w, h, x, y))
        
    def initUI(self):
        self.loadOldConfig()
        self.loadInitPoses()
        
        self.parent.title("MRP Experiment Launcher")
        self.style = Style()
        self.style.theme_use("alt")
        self.parent.resizable(width=FALSE, height=FALSE)
        self.pack(fill=BOTH, expand=1)
        self.centerWindow()

        self.columnconfigure(1, weight=1)
        self.columnconfigure(3, pad=7)
        self.rowconfigure(3, weight=1)
        self.rowconfigure(5, pad=7)
        
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
        tk.OptionMenu(self,self.robots_ddm, *self.robots_n_list).grid(sticky=W, row=1, column=1, pady=4, padx=5)

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
        tk.OptionMenu(self,self.alg_ddm, *self.algorithm_list).grid(sticky=W, row=2, column=1, pady=4, padx=5)
        
        
        launchButton = Button(self, text="Start Experiment",command=self.launch_script)
        launchButton.grid(sticky=W, row=3, column=0, pady=1, padx=5)
        
        launchButton = Button(self, text="Stop Experiment",command=self.kill_demo)
        launchButton.grid(sticky=W, row=3, column=1, pady=1, padx=5)
        
    
    def launch_script(self):
        MAP = self.map_ddm.get()
        NROBOTS = self.robots_ddm.get()
        ALG_SHORT = self.alg_ddm.get()
        ALG = findAlgName(ALG_SHORT)
        print 'Launch script'
        print 'Loading map ',MAP
        print 'N. robot ',NROBOTS
        print 'Algorithm ',ALG,"  ",ALG_SHORT
        self.saveConfigFile()
        scenario = MAP+"_"+NROBOTS
        print scenario,'   ',self.initPoses[scenario]
        
        os.system('xterm -e roscore &')
        os.system('sleep 3')
        os.system('rosparam set /use_sim_time true')
        cmd = 'rosrun patrolling_sim monitor maps/'+MAP+'/'+MAP+'.graph '+ALG_SHORT+' '+NROBOTS        
        print cmd
        os.system('xterm -e  "'+cmd+'" &')
        os.system('sleep 3')
        
        cmd = './setinitposes.py '+MAP+' "'+self.initPoses[scenario]+'"'
        print cmd
        os.system(cmd)
        os.system('sleep 1')
        
        cmd='roslaunch patrolling_sim map.launch map:='+MAP
        print cmd
        os.system('xterm -e  "'+cmd+'" &')
        
        # Start robots
        gcmd = 'gnome-terminal '
        for i in range(0,int(NROBOTS)):
            print 'Run robot ',i
            cmd = 'roslaunch patrolling_sim robot.launch robotname:=robot_'+str(i)+' mapname:='+MAP  #+' scenario:='+MAP+'_'+NROBOTS+'robots'
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
                cmd = 'rosrun patrolling_sim '+ALG+' __name:=patrol_robot'+str(i)+' maps/'+MAP+'/'+MAP+'.graph MSP/'+MAP+'/'+MAP+'_'+str(NROBOTS)+'_'+str(i)+' '+str(i)
            elif (ALG_SHORT=='GBS' or ALG_SHORT=='SEBS'):
                cmd = 'rosrun patrolling_sim '+ALG+' __name:=patrol_robot'+str(i)+' maps/'+MAP+'/'+MAP+'.graph '+str(i)+' '+str(NROBOTS)
            else:
                cmd = 'rosrun patrolling_sim '+ALG+' __name:=patrol_robot'+str(i)+' maps/'+MAP+'/'+MAP+'.graph '+str(i)
            print cmd
            #os.system('xterm -e  "'+cmd+'" &')
            #os.system('sleep 1')
            gcmd = gcmd + ' --tab -e "'+cmd+'" '
        gcmd = gcmd + '&'    
        print gcmd
        os.system(gcmd)
        os.system('sleep 5')
                
    def quit(self):
      self.parent.destroy()
      
    def kill_demo(self):
      os.system("./stop_experiment.sh")
      
      
    def saveConfigFile(self):
      f = open('lastConfigUsed', 'w')
      f.write("[Config]\n")
      f.write("map: %s\n"%self.map_ddm.get())
      f.write("nrobots: %s\n"%self.robots_ddm.get())
      f.write("algorithm: %s\n"%self.alg_ddm.get())
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

    def loadInitPoses(self):
      try:
        self.initPoses = {}
        self.ConfigIP = ConfigParser.ConfigParser()
        self.ConfigIP.read("params/initial_poses.txt")
        for option in self.ConfigIP.options("InitialPoses"):
          self.initPoses[option] = self.ConfigIP.get("InitialPoses", option)
      except:
        print "Could not load initial poses file"


def main():

    root = tk.Tk()
    DIP(root)
    root.geometry("300x200+0+0")
    root.mainloop()  


if __name__ == '__main__':
    main()
