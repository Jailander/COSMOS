#!/usr/bin/env python

import signal
import numpy as np
import rospy

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

class SimpleDataVisualiser(KrigingVisualiser):

    modes = {
        'help': 'press \'h\' for help',
        'none': ''
    }

    coords=[[53.13986, 0.00572], [53.13943, 0.00462], [53.13936, 0.00454], 
            [53.13917, 0.004], [53.13917, 0.00394], [53.13919, 0.00393], 
            [53.13919, 0.00391], [53.13916, 0.00383], [53.13893, 0.00324], 
            [53.1389, 0.00343], [53.1391, 0.00462], [53.13907, 0.00419], [53.13701, 0.00499]]

    def __init__(self, field_file, size, cell_size):
        self.running = True
        signal.signal(signal.SIGINT, self.signal_handler)


#        with open(field_file, 'r') as f:
#            a = yaml.load(f)


        lat_deg = self.coords[0][0]
        lon_deg = self.coords[0][1]
        zoom = 16#a['field']['zoom']

        print "Creating visualiser object"
        super(SimpleDataVisualiser, self).__init__(lat_deg, lon_deg, zoom, size)

        cv2.namedWindow('SimpleDataVisualiser')

        self.gps_canvas = ViewerCanvas(self.base_image.shape, self.satellite.centre, self.satellite.res)
        self.draw_coords()
        
        while(self.running):
            self.refresh()
            cv2.imshow('SimpleDataVisualiser', self.image)
            k = cv2.waitKey(20) & 0xFF
#            self._change_mode(k)
            if k == 27:
                self.running = False
            rospy.sleep(0.2)
            
        cv2.destroyAllWindows()       
        sys.exit(0)


    def draw_coords(self):
        colours=['red','blue','green','magenta','cyan','yellow']
        cind=0
        for i in self.coords:
            self.dgps_coord = MapCoords(i[0],i[1])
            self.gps_canvas.draw_coordinate(self.dgps_coord,colours[cind],size=4, thickness=2, alpha=255)
            cind+=1
            if cind>=len(colours):
                cind=0

    def refresh(self):
        self.image = cv2.addWeighted(self.gps_canvas.image, 0.3, self.image, 1.0, 0)


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

