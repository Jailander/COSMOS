#!/usr/bin/env python

import signal
import numpy as np

import cv2
import sys
import yaml
import argparse

import matplotlib as mpl
import matplotlib.cm as cm



from kriging_exploration.visualiser import KrigingVisualiser
from kriging_exploration.data_grid import DataGrid


class SimpleDataVisualiser(KrigingVisualiser):

    modes = {
        'help': 'press \'h\' for help',
        'none': ''
    }

    def __init__(self, field_file, size, cell_size):
        self.running = True
        signal.signal(signal.SIGINT, self.signal_handler)

        with open(field_file, 'r') as f:
            a = yaml.load(f)


        lat_deg = a['field']['lat']
        lon_deg = a['field']['lon']
        zoom =a['field']['zoom']

        print "Creating visualiser object"
        super(SimpleDataVisualiser, self).__init__(lat_deg, lon_deg, zoom, size)
#
        cv2.namedWindow('SimpleDataVisualiser')
#
#        self.grid = DataGrid('limits.coords', cell_size)
#        
#        self.image = self.satellite.base_image.copy()
#        
#        
        while(self.running):
            cv2.imshow('SimpleDataVisualiser', self.image)
            k = cv2.waitKey(20) & 0xFF
#            self._change_mode(k)
            if k == 27:
                self.running = False
        cv2.destroyAllWindows()       
        sys.exit(0)


    def signal_handler(self, signal, frame):
        cv2.destroyAllWindows()
        print('You pressed Ctrl+C!')
        sys.exit(0)

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("--field_file", type=str, default='footbal.field',
                        help="cell size in meters")
    parser.add_argument("--cell_size", type=int, default=10,
                        help="field definition file")
    args = parser.parse_args()
    
    SimpleDataVisualiser(args.field_file, 640, args.cell_size)
#    SimpleDataVisualiser(53.261685, -0.525158, 17, 640, args.cell_size)

