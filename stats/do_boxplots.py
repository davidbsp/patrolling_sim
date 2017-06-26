#!/usr/bin/env python
#
# GIT version. Please do not edit this file.
# Make a copy and edit the copy.
#

import sys
import os


# L = <list of experiments to compare and label to use in the plot (no spaces allowed in the label!!!)>

L = []


# Read file with experiments to plot

if (len(sys.argv)<4):
    print "Use: do_boxplots.py <experiments_file> <plot_title> <outfilename_prefix> <label_1> ... <label_n>"
    sys.exit(0)

exptoplot = sys.argv[1]
plot_title = sys.argv[2]
outfile = sys.argv[3]

k = 4; # index for the labels

#print "File with exp to plot: ",exptoplot

f = open(exptoplot,'r')

for line in f:
    e = line.rstrip('\r\n ')
    if (len(e)>0 and e[0]!='#'):
        L.append(e)
        L.append(sys.argv[k])
        k=k+1

f.close

# Run the R script

cmd = 'Rscript stats/boxplots.R "'+plot_title+'" '
lfiles = ""

for i in range(0,len(L)/2):
  lfiles = lfiles + "%s_idleness.csv %s " %(L[i*2],L[i*2+1])

cmd = cmd + lfiles
print cmd
os.system(cmd)

cmd = 'mv Rplots.pdf '+outfile+"_boxplots.pdf"
os.system(cmd)

