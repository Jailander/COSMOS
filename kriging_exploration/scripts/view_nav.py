#!/usr/bin/env python


import cv2
import sys

import signal
import numpy as np
import utm

import rospy

from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry


from kriging_exploration.map_coords import MapCoords
from kriging_exploration.visualiser import KrigingVisualiser

from kriging_exploration.canvas import ViewerCanvas

class SimpleDataVisualiser(KrigingVisualiser):

    def __init__(self, zoom, size, cell_size):
        self.running = True
        signal.signal(signal.SIGINT, self.signal_handler)

        initial_gps = rospy.wait_for_message("/navsat_fix", NavSatFix, 10)
        self.initial_odom = rospy.wait_for_message("/odometry", Odometry, 10)
        self.initial_godom = rospy.wait_for_message("/gps_odom", Odometry, 10)        
        
        print initial_gps


        super(SimpleDataVisualiser, self).__init__(initial_gps.latitude, initial_gps.longitude, zoom, size)

        self.gps_canvas = ViewerCanvas(self.base_image.shape, self.satellite.centre, self.satellite.res)
        self.compass_canvas = ViewerCanvas(self.base_image.shape, self.satellite.centre, self.satellite.res)
        
        self.preodom= self.centre                
        
        rospy.loginfo("Subscribing to GPS Data")
        rospy.Subscriber("/navsat_fix", NavSatFix, self.gps_callback)
        rospy.Subscriber("/fix", NavSatFix, self.mti_callback)
        rospy.Subscriber("/odometry", Odometry, self.odom_callback)
        rospy.Subscriber("/gps_odom", Odometry, self.godom_callback)        
        
        while(self.running):
            self.refresh()
            cv2.imshow('image', self.image)
            k = cv2.waitKey(20) & 0xFF
            self._change_mode(k)


    def godom_callback(self, data):
        d = utm.to_latlon(data.pose.pose.position.x, data.pose.pose.position.y, self.centre.zone_number, zone_letter=self.centre.zone_letter)
        
        odom_coord = MapCoords(d[0],d[1])
             
        diff = odom_coord - self.preodom
        
        self.draw_compass(diff)
        self.preodom = odom_coord
        self.gps_canvas.draw_coordinate(odom_coord, 'yellow',size=2, thickness=1, alpha=255)
        #self.draw_coordinate(odom_coord.lat, odom_coord.lon,'yellow',size=2, thickness=1, alpha=255)

    def draw_compass(self, diff):
        print diff
        self.compass_canvas.clear_image()
        compass_str= str(diff[1])
        self.compass_canvas.put_text(compass_str)
        
    def refresh(self):
        self.image = cv2.addWeighted(self.compass_canvas.image, 0.5, self.base_image, 1.0, 0)
        self.image = cv2.addWeighted(self.gps_canvas.image, 0.3, self.image, 1.0, 0)


    def odom_callback(self, data):
        dx = data.pose.pose.position.x - self.initial_odom.pose.pose.position.x
        dy = data.pose.pose.position.y - self.initial_odom.pose.pose.position.y
        #print dx, dy
        odom_coord = self.centre._get_rel_point(-dy, dx)
        self.gps_canvas.draw_coordinate(odom_coord,'green',size=2, thickness=1, alpha=255)
        

    def gps_callback(self, data):
        if not np.isnan(data.latitude):
            gps_coord = MapCoords(data.latitude,data.longitude)
            self.gps_canvas.draw_coordinate(gps_coord,'red',size=2, thickness=1, alpha=255)
#            self.draw_coordinate(data.latitude, data.longitude,'red',size=2, thickness=1, alpha=255)

    def mti_callback(self, data):
        if not np.isnan(data.latitude):
            gps_coord = MapCoords(data.latitude,data.longitude)
            self.gps_canvas.draw_coordinate(gps_coord,'white',size=2, thickness=1, alpha=255)
#            self.draw_coordinate(data.latitude, data.longitude,'white',size=2, thickness=1, alpha=255)

    def _change_mode(self, k):
        if k == 27:
            self.running = False
        elif k == ord('q'):
            self.running = False


    def signal_handler(self, signal, frame):
        cv2.destroyAllWindows()
        print('You pressed Ctrl+C!')
        sys.exit(0)


if __name__ == '__main__':   
    rospy.init_node('kriging_exploration')
    SimpleDataVisualiser(17, 640, 10)
