#!/usr/bin/env python

import signal
import yaml
import sys
import argparse
import kriging_exploration.canvas

from kriging_exploration.map_coords import MapCoords
from kriging_exploration.canvas import ViewerCanvas
from kriging_exploration.visualiser import KrigingVisualiser



import numpy as np

import cv2

import matplotlib as mpl
import matplotlib.cm as cm

import rospy

import std_msgs

from cosmos_msgs.msg import KrigInfo
from cosmos_msgs.msg import KrigMsg
from cosmos_msgs.srv import CompareModels

#import satellite
from kriging_exploration.satellite import SatelliteImage


#from krigging_data import KriggingData
from sensor_msgs.msg import NavSatFix




def load_field_defs(filename):
    with open(filename, 'r') as f:
        a = yaml.load(f)
    return a

class PoissonSimulation(KrigingVisualiser):

    modes = {
        'help': 'press \'h\' for help',
        'none': ''
    }

    def __init__(self, field, cell_size):
        self.running = True
        signal.signal(signal.SIGINT, self.signal_handler)


        print "Creating visualiser object"
        super(PoissonSimulation, self).__init__(field['field']['lat'], field['field']['lon'], field['field']['zoom'], 640)

        limits=[]
        for i in field['field']['limits']:
            b=MapCoords(i[0],i[1])
            limits.append(b)

        cv2.namedWindow('SimpleDataVisualiser')
        self.image = self.satellite.base_image.copy()

        self.create_grid(cell_size, limits)
        self.draw_grid()



        self.load_groundtruth('airfield-sim.data')
        self.grid.krieg_all_mmodels()

        self.redraw()
#        self.drawing_grid(self.centre, -15.5)
#        self.generate_files()
        while(self.running):
            cv2.imshow('SimpleDataVisualiser', self.image)
            k = cv2.waitKey(20) & 0xFF
#            self._change_mode(k)
            if k == 27:
                self.running = False
        cv2.destroyAllWindows()       
        sys.exit(0)



    def load_groundtruth(self, filename):
        self.grid.load_data_from_yaml(filename)
        self.vmin, self.vmax = self.grid.get_max_min_vals()
        print "LIMS: " + str(self.vmin) + " " + str(self.vmax)
        self.n_models=len(self.grid.models)
        self.current_model=0


    def redraw(self):
        self.image = kriging_exploration.canvas.overlay_image_alpha(self.image,self.grid_canvas.image)


    def signal_handler(self, signal, frame):
        cv2.destroyAllWindows()
        print('You pressed Ctrl+C!')
        sys.exit(0)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("--cell_size", type=int, default=10,
                        help="cell size in meters")
    parser.add_argument("--field_def_file", type=str, default="field.field",
                        help="cell size in meters")
    args = parser.parse_args()
    
    field=load_field_defs(args.field_def_file)
    rospy.init_node('kriging_simulator')
    PoissonSimulation(field, args.cell_size)

   
#    simulator(53.267213, -0.533420, 17, 640, args.cell_size)  #Football Field
#    simulator(39.432305, 22.79269, 17, 640, args.cell_size)    #Greece
#    simulator(53.138604, 0.004182, 19, 640, args.cell_size)    #Greece    
#    simulator(53.261576, -0.526648, 17, 640, args.cell_size)  #Half cosmos field
#    simulator(53.261685, -0.525158, 17, 640, args.cell_size)  #Full cosmos field

