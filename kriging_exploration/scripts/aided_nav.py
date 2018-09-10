#!/usr/bin/env python

import signal
import yaml
import sys
import argparse
import rospy
import random

import cv2
import actionlib


import open_nav.msg
import kriging_exploration.canvas

from kriging_exploration.map_coords import MapCoords
from kriging_exploration.canvas import ViewerCanvas
from kriging_exploration.visualiser import KrigingVisualiser



import numpy as np
import math

import std_msgs


from kriging_exploration.topological_map import TopoMap


#import satellite
#from kriging_exploration.satellite import SatelliteImage


#from krigging_data import KriggingData
from sensor_msgs.msg import NavSatFix




def load_field_defs(filename):
    with open(filename, 'r') as f:
        a = yaml.load(f)
    return a

class AidedNAv(KrigingVisualiser):

    modes = {
        'help': 'press \'h\' for help',
        'none': ''
    }
    running=True
    navigating=False
    def __init__(self, field, cell_size):

        signal.signal(signal.SIGINT, self.signal_handler)

        print "Creating visualiser object"
        super(AidedNAv, self).__init__(field['field']['lat'], field['field']['lon'], field['field']['zoom'], 640)

        limits=[]
        for i in field['field']['limits']:
            b=MapCoords(i[0],i[1])
            limits.append(b)

        cv2.namedWindow('aided_nav')
        cv2.setMouseCallback('aided_nav', self.click_callback)
        
        self.image = self.satellite.base_image.copy()

        self.create_grid(cell_size, limits)
        self.draw_grid()
        self.topo_map= TopoMap(self.grid)
        self.draw_topo_map(self.topo_map.waypoints)

        rospy.Subscriber("/navsat_fix", NavSatFix, self.gps_callback)
        self.add_gps_canvas()

        #rospy.loginfo(" ... Connecting to Open_nav")
        self.nav_canvas = ViewerCanvas(self.base_image.shape, self.satellite.centre, self.satellite.res)
        
        drawtim = rospy.Timer(rospy.Duration(0.1), self.draw_timer_callback)


        self.redraw()
        self.refresh()
        while(self.running):
            cv2.imshow('aided_nav', self.show_image)
            k = cv2.waitKey(20) & 0xFF
            self._change_mode(k)
            if k == 27:
                self.running = False
#            rospy.sleep(0.05)

        drawtim.shutdown()
        cv2.destroyAllWindows()       
        sys.exit(0)


    def redraw(self):
        self.image = self.satellite.base_image.copy()
        self.image = kriging_exploration.canvas.overlay_image_alpha(self.image,self.grid_canvas.image)
        self.image = kriging_exploration.canvas.overlay_image_alpha(self.image,self.topo_canvas.image)
        self.image = kriging_exploration.canvas.overlay_image_alpha(self.image,self.nav_canvas.image)
        
    def _change_mode(self, k):
        if k == 27:
            self.running = False
        elif k == ord('q'):
            self.running = False
        elif k == ord('w'):
            for i in self.topo_map.waypoints:
                print i.name

    def draw_timer_callback(self, event):
        self.refresh()

    def refresh(self):
        self.show_image = kriging_exploration.canvas.overlay_image_alpha(self.image,self.gps_canvas.image)
        #self.show_image = kriging_exploration.canvas.overlay_image_alpha(self.image,self.nav_canvas.image)

    def navigate_to(self, coord):
        #print coord
        self.nav_canvas.clear_image()
        self.nav_canvas.draw_coordinate(coord.coord,'blue',size=5, thickness=2, alpha=255)
        self.nav_canvas.put_text(coord.name,colour=(230,0,0,255), text_size=0.8, x_or=40, y_or=40)
        self.goal_coord=coord.coord
        self.navigating=True
#        draw_coordinate(coord,'blue',size=5, thickness=2, alpha=255)
        self.redraw()

    def click_callback(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            click_coord = self.satellite._pix2coord(x,y)
            cx, cy = self.grid.get_cell_inds_from_coords(click_coord)

            if cx <0 or cy<0:
                print "click outside the grid"
                self.navigating=False
            else:
                for i in self.topo_map.waypoints:
                    if (cy,cx) == i.ind:
                        self.navigate_to(i)


    def gps_callback(self, data):
        if not np.isnan(data.latitude):
            self.gps_canvas.clear_image()
            self.gps_coord = MapCoords(data.latitude,data.longitude)            
            self.gps_canvas.draw_coordinate(self.gps_coord,'red',size=2, thickness=2, alpha=255)
            if self.navigating:
                dn= self.gps_coord.northing - self.goal_coord.northing
                de= self.gps_coord.easting - self.goal_coord.easting
                if dn>0:
                    dn=(math.ceil(dn*100)/100)
                    dnstr= str(dn)+"m S"
                else:
                    dn=(math.ceil((-1.0*dn)*100)/100)
                    dnstr= str(dn)+"m N"

                if de>0:
                    de=(math.ceil(de*100)/100)
                    destr= str(de)+"m W"
                else:
                    de=(math.ceil((-1.0*de)*100)/100)
                    destr= str(de)+"m E"

                self.gps_canvas.put_text(dnstr,colour=(0,0,230,255), text_size=0.7, x_or=40, y_or=600) 
                self.gps_canvas.put_text(destr,colour=(0,0,230,255), text_size=0.7, x_or=360, y_or=600)
        
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
    rospy.init_node('aided_navigation')
    AidedNAv(field, args.cell_size)

