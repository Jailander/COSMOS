#!/usr/bin/env python

import signal
import numpy as np
import rospy
import time
import math

import cv2
import sys
import yaml
import argparse

import matplotlib as mpl
import matplotlib.cm as cm

import matplotlib.pyplot as plt

from sensor_msgs.msg import NavSatFix

from kriging_exploration.map_coords import MapCoords

from kriging_exploration.visualiser import KrigingVisualiser
from kriging_exploration.data_grid import DataGrid
from kriging_exploration.canvas import ViewerCanvas

from kriging_exploration.topological_map import TopoMap


def overlay_image_alpha(img, img_overlay):
    """Overlay img_overlay on top of img at the position specified by
    pos and blend using alpha_mask.
    """
    show_image = img.copy()
    alpha =  img_overlay[:, :, 3] / 255.0   # Alpha mask must contain values 
                                            # within the range [0, 1] 
                                            # and be the same size as img_overlay.
    # Image ranges
    y1, y2 = 0, img.shape[0]
    x1, x2 = 0, img.shape[1]

    channels = img.shape[2]

    alpha_inv = 1.0 - alpha

    for c in range(channels):
        show_image[y1:y2, x1:x2, c] = (alpha * img_overlay[y1:y2, x1:x2, c] + alpha_inv * img[y1:y2, x1:x2, c])
    return show_image



class topo_map_node(object):
    def __init__(self, name, coord):
        self.name = name
        self.coord = coord


    def __repr__(self):
        a = dir(self)
        b = []
        s = ''
        for i in a:
            if not i.startswith('_'):
                b.append(str(i))
        for i in b:
                s = s + str(i) + ': ' + str(self.__getattribute__(i)) + '\n'
        return s


class SimpleDataVisualiser(KrigingVisualiser):

    modes = {
        'help': 'press \'h\' for help',
        'none': ''
    }


    def __init__(self, field_file, size, cell_size):
        self.running = True
        self.topo_map=[]
        signal.signal(signal.SIGINT, self.signal_handler)
        #map_angle=30.0

        with open(field_file, 'r') as f:
            a = yaml.load(f)

        print a
        lat_deg = a['field']['lat']
        lon_deg = a['field']['lon']
        zoom = a['field']['zoom']

        print "Creating visualiser object"
        super(SimpleDataVisualiser, self).__init__(lat_deg, lon_deg, zoom, size)

        cv2.namedWindow('SimpleDataVisualiser')
        cv2.setMouseCallback('SimpleDataVisualiser', self.click_callback)

        self.grid = DataGrid(a['field']['limits_file'], 20)
#        self.grid2 = DataGrid(a['field']['limits_file'], 10)
#        self.load_groundtruth()
        self.topo_map= TopoMap(self.grid)

        self.map_canvas = ViewerCanvas(self.base_image.shape, self.satellite.centre, self.satellite.res)
        self.gps_canvas = ViewerCanvas(self.base_image.shape, self.satellite.centre, self.satellite.res)        
        
                
        rospy.loginfo("Subscribing to GPS Data")
        rospy.Subscriber("/fix", NavSatFix, self.gps_callback)
    
        print "Draw Grid"
        print "Grid Shape: ", self.grid.shape
        self.map_canvas.draw_grid(self.grid.cells, self.grid.cell_size, (64,64,64,64), thickness=1)
        self.map_canvas.draw_polygon(self.grid.limits, 'yellow', thickness=2)
        self.draw_topo_map(self.topo_map.waypoints, colour='red', size=3, thickness=2)
        self.redraw()
        
        print self.topo_map.waypoints[-1]        
        
        while(self.running):
            self.refresh()
            cv2.imshow('SimpleDataVisualiser', self.show_image)
            k = cv2.waitKey(20) & 0xFF
#            self._change_mode(k)
            if k == 27:
                self.running = False
            time.sleep(0.02)
            
        cv2.destroyAllWindows()       
        sys.exit(0)

       


    def click_callback(self, event, x, y, flags, param):
        
        if event == cv2.EVENT_LBUTTONDOWN:
            click_coord = self.satellite._pix2coord(x,y)
            closest_click = self.get_closest_node(click_coord)
            print "Closest CLICK: ", closest_click



    def gps_callback(self, data):
        if not np.isnan(data.latitude):
            gps_coord = MapCoords(data.latitude,data.longitude)
            self.closest_node = self.get_closest_node(gps_coord)
            self.gps_canvas.clear_image()
            self.gps_canvas.draw_coordinate(self.closest_node.coord,'red',size=6, thickness=2, alpha=255)
            self.gps_canvas.draw_coordinate(gps_coord,'white',size=2, thickness=2, alpha=255)



    def redraw(self):
        #self.image = cv2.addWeighted(self.gps_canvas.image, 0.3, self.image, 1.0, 0)
        self.image = overlay_image_alpha(self.image, self.topo_canvas.image)
        self.image = overlay_image_alpha(self.image, self.map_canvas.image)


    def refresh(self):
        #self.show_image = cv2.addWeighted(self.gps_canvas.image, 0.3, self.image, 1.0, 0)
        self.show_image = overlay_image_alpha( self.image, self.gps_canvas.image)

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

    rospy.init_node('topomap_vis')
    SimpleDataVisualiser(args.field_file, 640, args.cell_size)
#    SimpleDataVisualiser(53.261685, -0.525158, 17, 640, args.cell_size)

