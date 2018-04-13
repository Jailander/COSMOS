#!/usr/bin/env python

import yaml
import numpy as np
import copy


import argparse

#import matplotlib.pyplot as plt
#from matplotlib.ticker import MaxNLocator
#import matplotlib as mpl
#import matplotlib.cm as cm

#import rospy


#import satellite
#from kriging_exploration.satellite import SatelliteImage
from kriging_exploration.map_coords import MapCoords
from kriging_exploration.data_grid import DataGrid
#from krigging_data import KriggingData


#
#def data_to_yaml(data_fn):
#    mods = {}
##    mods['names']=['0.0 cm', '2.5 cm' ,'5.0 cm', '7.5 cm', '10.0 cm', '12.5 cm', '15.0 cm', '17.5 cm', '20.0 cm', '22.5 cm' ,'25.0 cm', '27.5 cm', '30.0 cm', '32.5 cm', '35.0 cm', '37.5 cm', '40.0 cm', '42.5 cm', '45.0 cm']
#    mods['data']=[]
##    vals=[]
#    print "open: " + data_fn
#    f = open(data_fn, 'r')
#    for line in f:
#        line=line.strip('\n')
#        a=line.split(';')
#        val={}
#        val['position']={'lat':float(a[0]), 'lon':float(a[1])}
#        val['data']= [float(i) for i in a[2:]]
#        mods['data'].append(val)
#    
#    print mods['data']
#    yml = yaml.safe_dump(mods, default_flow_style=False)
#    print yml
#    filename = data_fn+'.yaml'
#    fh = open(filename, "w")
#    s_output = str(yml)
#    fh.write(s_output)
#    fh.close


class gt_centre(object):

    def __init__(self, cell_size):
        self.running = True
        
        self.grid = DataGrid('football.coords', cell_size)
        #self.grid.load_data_from_yaml('testing-5cm-intervals.yaml')       
        self.load_groundtruth('football-field.yaml')
        

    def load_groundtruth(self, filename):
        with open(filename, 'r') as f:
            a = yaml.load(f)
            
        b = copy.deepcopy(a)
        for i in range(len(b['data'])):
            #print "-------------------"
            #print i['position']
            bc = MapCoords(b['data'][i]['position']['lat'],b['data'][i]['position']['lon'])
            cx, cy = self.grid.get_cell_inds_from_coords(bc)
            #print self.grid.cells[cy][cx]
            b['data'][i]['position']['lat']=self.grid.cells[cy][cx].lat
            b['data'][i]['position']['lon']=self.grid.cells[cy][cx].lon
            #print i['position']            
    
        for i in range(len(b['data'])):
            print "-------------------"
            print a['data'][i]['position']
            print b['data'][i]['position']
    
        outfile = 'football-centered.yaml'            
        yml = yaml.safe_dump(b, default_flow_style=False)
        fh = open(outfile, "w")
        s_output = str(yml)
        print s_output
        fh.write(s_output)
        fh.close
    
    

            

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("--cell_size", type=int, default=10,
                        help="cell size in meters")
    args = parser.parse_args()
    

    gt_centre(args.cell_size)  #Half cosmos field


