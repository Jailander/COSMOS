#!/usr/bin/env python

import signal
import numpy as np
import rospy
import time

import cv2
import sys
import yaml
import argparse

import matplotlib as mpl
import matplotlib.cm as cm

from kriging_exploration.map_coords import MapCoords

from kriging_exploration.visualiser import KrigingVisualiser
from kriging_exploration.data_grid import DataGrid
from kriging_exploration.canvas import ViewerCanvas

class GTDataVisualiser(KrigingVisualiser):

    modes = {
        'help': 'press \'h\' for help',
        'none': ''
    }

    def __init__(self, field_file, size, cell_size, gt_file):
        self.running = True
        signal.signal(signal.SIGINT, self.signal_handler)


        with open(field_file, 'r') as f:
            a = yaml.load(f)

        print a
        lat_deg = a['field']['lat']
        lon_deg = a['field']['lon']
        zoom = a['field']['zoom']

        print "Creating visualiser object"
        super(GTDataVisualiser, self).__init__(lat_deg, lon_deg, zoom, size)

        cv2.namedWindow('Groundtruth')

        self.grid = DataGrid(a['field']['limits_file'], 10)
        self.load_groundtruth(gt_file)


        self.gps_canvas = ViewerCanvas(self.base_image.shape, self.satellite.centre, self.satellite.res)

        
        while(self.running):
            self.refresh()
            cv2.imshow('Groundtruth', self.image)
            k = cv2.waitKey(20) & 0xFF
            if k == 27:
                self.running = False
            time.sleep(0.2)
            
        cv2.destroyAllWindows()       
        sys.exit(0)




    def load_groundtruth(self, gt_file):
        self.grid.load_data_from_yaml(gt_file)


    def refresh(self):
        self.image = cv2.addWeighted(self.gps_canvas.image, 0.3, self.image, 1.0, 0)

    def signal_handler(self, signal, frame):
        cv2.destroyAllWindows()
        print('You pressed Ctrl+C!')
        sys.exit(0)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("--field_file", type=str, default='field.field',
                        help="field definition file")
    parser.add_argument("--gt_file", type=str, default='gt.yaml',
                        help="groundtruth data file")
    parser.add_argument("--cell_size", type=int, default=10,
                        help="cell size in meters")
    args = parser.parse_args()
    
    GTDataVisualiser(args.field_file, 640, args.cell_size, args.gt_file)


