#!/usr/bin/env python

import glob
import os

import sys
import yaml




if __name__ == '__main__':
    filenames=[]
    for fil in glob.glob("*.exp"):
        filenames.append(fil)

    filenames.sort()    
    
#    field_def_file='airfieldtestfile.field'
    field_def_file='Greece.field'    
    cell_size = 5
    for i in filenames:
        cmdstr = ('rosrun kriging_exploration poisson_explorer.py --field_def_file %s --cell_size %d --exp_def_file %s --automatic true --timeout %d' 
                    %(field_def_file, cell_size, i, 3600))
        os.system(cmdstr)
        print i
