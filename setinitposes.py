#!/usr/bin/env python
#

import sys, time, os
from math import degrees, radians

colors = ['blue', 'red', 'cyan', 'magenta', 'blue', 'red', 'cyan', 'magenta', 'blue', 'red', 'cyan', 'magenta']

def setIP(mapname,vip):
    cmd = 'rosparam set initial_pos "'+vip+'"'
    print cmd
    os.system(cmd)
    ip = eval(vip)
    n = len(ip)/2
    print 'Set initial poses of ',n,' robots'

    fnr = 'maps/'+mapname+'/robots.inc'
    fr = open(fnr, 'w')

    for i in range(0,n):
        x = ip[i*2]
        y = ip[i*2+1]
        th = 90
        fn = 'params/amcl/robot_'+str(i)+'_initial_pose.xml'
        f = open(fn, 'w')
        f.write('<launch>\n')
        f.write('  <param name="amcl/initial_pose_x" value="'+str(x)+'"/>\n')
        f.write('  <param name="amcl/initial_pose_y" value="'+str(y)+'"/>\n')
        f.write('  <param name="amcl/initial_pose_a" value="'+str(radians(th))+'"/>\n')
        f.write('  <param name="srrg_localizer/initial_pose_x" value="'+str(x)+'"/>\n')
        f.write('  <param name="srrg_localizer/initial_pose_y" value="'+str(y)+'"/>\n')
        f.write('  <param name="srrg_localizer/initial_pose_a" value="'+str(radians(th))+'"/>\n')
        f.write('</launch>\n')
        f.close()

        fr.write('crobot( pose [ '+str(x)+'  '+str(y)+'  0  '+str(th)+' ]   name "robot'+str(i)+'"  color "'+colors[i]+'")\n')
        

    if (n==1):
        # inactive robot for having correct namespaces
        fr.write('crobot( pose [ -2.0   -2.0    0   0.0 ]   name "robot1"  color "red")\n')

    fr.close()



if __name__ == '__main__':
    if (len(sys.argv)<3):
        sys.exit(0)
    mapname = sys.argv[1]
    vip = sys.argv[2]
    setIP(mapname,vip)
