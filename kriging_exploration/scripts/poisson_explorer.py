#!/usr/bin/env python

import signal
import yaml
import sys
import argparse
import rospy

import cv2
import actionlib


import open_nav.msg
import kriging_exploration.canvas

from kriging_exploration.map_coords import MapCoords
#from kriging_exploration.canvas import ViewerCanvas
from kriging_exploration.visualiser import KrigingVisualiser



import numpy as np


import std_msgs

from cosmos_msgs.msg import KrigInfo
from cosmos_msgs.msg import KrigMsg
from cosmos_msgs.srv import CompareModels

#import satellite
#from kriging_exploration.satellite import SatelliteImage


#from krigging_data import KriggingData
from sensor_msgs.msg import NavSatFix




def load_field_defs(filename):
    with open(filename, 'r') as f:
        a = yaml.load(f)
    return a

class PoissonExploration(KrigingVisualiser):

    modes = {
        'help': 'press \'h\' for help',
        'none': ''
    }
    draw_mode="none"
    current_model=0
    running = True
    publish = False
    current_counts=0
    number_of_messages=0
    
    def __init__(self, field, cell_size):

        signal.signal(signal.SIGINT, self.signal_handler)

        print "Creating visualiser object"
        super(PoissonExploration, self).__init__(field['field']['lat'], field['field']['lon'], field['field']['zoom'], 640)

        limits=[]
        for i in field['field']['limits']:
            b=MapCoords(i[0],i[1])
            limits.append(b)

        cv2.namedWindow('poisson_explorer')
        cv2.setMouseCallback('poisson_explorer', self.click_callback)
        
        self.image = self.satellite.base_image.copy()

        self.create_grid(cell_size, limits)
        self.draw_grid()


        rospy.Subscriber('/kriging_data', KrigInfo, self.scan_callback)
        rospy.Subscriber("/navsat_fix", NavSatFix, self.gps_callback)
        self.req_data_pub = rospy.Publisher('/request_scan', std_msgs.msg.String, latch=False, queue_size=1)
        self.add_gps_canvas()

        rospy.loginfo(" ... Connecting to Open_nav")
        
        self.open_nav_client = actionlib.SimpleActionClient('/open_nav', open_nav.msg.OpenNavAction)
        self.open_nav_client.wait_for_server()


        drawtim = rospy.Timer(rospy.Duration(0.03), self.draw_timer_callback)

#        self.grid.krieg_all_mmodels()
#        self.draw_krigged(0, alpha=200)

        self.redraw()
        self.refresh()
        while(self.running):
            cv2.imshow('poisson_explorer', self.show_image)
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

        if self.draw_mode=='inputs':
            self.image = kriging_exploration.canvas.overlay_image_alpha(self.image,self.model_canvas[self.current_model].image)
        if self.draw_mode=='kriging':
            self.image = kriging_exploration.canvas.overlay_image_alpha(self.image,self.kriging_canvas[self.current_model].image)
        self.image = kriging_exploration.canvas.overlay_image_alpha(self.image,self.grid_canvas.image)
        
        
    def _change_mode(self, k):
        if k == 27:
            self.running = False
        elif k == ord('q'):
            self.running = False
        elif k == ord('n'):
            print len(self.grid.models)
            info_str='stop_reading'
            self.req_data_pub.publish(info_str)
            self.current_counts=0
            self.number_of_messages=0
        elif k == ord('i'):
            if self.n_models > 0:
                self.draw_mode="inputs"
                self.draw_inputs(0, alpha=200)
                self.current_model=0
                self.redraw()
        elif k == ord('k'):
            if self.n_models > 0:
                self.draw_mode="kriging"
                self.current_model=0
                self.redraw()


    def draw_timer_callback(self, event):
        self.refresh()

    def refresh(self):
        self.show_image = kriging_exploration.canvas.overlay_image_alpha(self.image,self.gps_canvas.image)


    def click_callback(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            click_coord = self.satellite._pix2coord(x,y)
            cx, cy = self.grid.get_cell_inds_from_coords(click_coord)

            if cx <0 or cy<0:
                print "click outside the grid"
            else:
                self.open_nav_client.cancel_goal()
                targ = open_nav.msg.OpenNavActionGoal()

                #goal.goal.goal.header.
                targ.goal.coords.header.stamp=rospy.Time.now()
                targ.goal.coords.latitude=click_coord.lat
                targ.goal.coords.longitude=click_coord.lon

                print targ
                self.navigating=True
                self.open_nav_client.send_goal(targ.goal)
                self.open_nav_client.wait_for_result()
                info_str='start_reading'
                self.req_data_pub.publish(info_str)


    def gps_callback(self, data):
        if not np.isnan(data.latitude):
            self.gps_canvas.clear_image()
            self.gps_coord = MapCoords(data.latitude,data.longitude)            
            self.gps_canvas.draw_coordinate(self.gps_coord,'black',size=2, thickness=2, alpha=255)
            
        
    def scan_callback(self, data):
        self.current_counts += data.data[0].measurement
        self.number_of_messages += 1
        print self.number_of_messages, self.current_counts, self.current_counts/self.number_of_messages, np.sqrt(self.current_counts), 100 * np.sqrt(self.current_counts)/self.current_counts


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
    rospy.init_node('poisson_explorer')
    PoissonExploration(field, args.cell_size)
