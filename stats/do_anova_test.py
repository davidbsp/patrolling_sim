#!/usr/bin/env python

import os

D = '../results/cumberland_4/GBS'

L = [
'vaio-i7/20160114_132727',
'vaio-i7/20160116_061244',
'vaio-i7/20160116_072029'
]

cmd = "Rscript anova_multi.R "

for i in range(0,len(L)):
  cmd = cmd + "%s/%s_idleness.txt " %(D,L[i])

print cmd
os.system(cmd)

