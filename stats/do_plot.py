#!/usr/bin/env python

import os

D = '../results/cumberland_4/'

L = [
D+'SEBS/vaio-i7/20160117_153630','SEBS',
D+'DTAP/vaio-i7/20160117_175721','DTAP',
]

cmd = "Rscript boxplots.R "
lfiles = ""

for i in range(0,len(L)/2):
  lfiles = lfiles + "%s_idleness.txt %s " %(L[i*2],L[i*2+1])

print cmd+lfiles
os.system(cmd+lfiles)
