#!/usr/bin/env python


import cv2
import sys

import signal
import numpy as np
import utm

import rospy

from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped

from kriging_exploration.map_coords import MapCoords
from kriging_exploration.visualiser import KrigingVisualiser

from kriging_exploration.canvas import ViewerCanvas




class SimpleDataVisualiser(KrigingVisualiser):

    def __init__(self, zoom, size, cell_size):
        self.dgps_coord=None
        self.mti_coord=None
        self.mti_coords=[]
        self.vels=[]
        self.pre_dgps_coord=None
        self.pre_mti_coord=None
        self.speed_coord=None
        self.pre_speed_coord=None
        self.running = True
        self.heading=0.0
        signal.signal(signal.SIGINT, self.signal_handler)

        initial_gps = rospy.wait_for_message("/navsat_fix", NavSatFix, 10)
        self.initial_odom = rospy.wait_for_message("/odometry", Odometry, 10)
        #self.initial_godom = rospy.wait_for_message("/gps_odom", Odometry, 10)        
        
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
        rospy.Subscriber("/vel", TwistStamped, self.vel_callback)  
        
        
        tim1 = rospy.Timer(rospy.Duration(1.0), self.speed_calc_timer)
        
        while(self.running):
            self.refresh()
            cv2.imshow('image', self.image)
            k = cv2.waitKey(20) & 0xFF
            self._change_mode(k)
            rospy.sleep(0.01)

        tim1.shutdown()


    def vel_callback(self, data):
        self.vels.append(data.twist.linear.x)
        

    def speed_calc_timer(self, event):
        self.speed_coord = self.average_coords()#self.mti_coord
        
        if self.pre_speed_coord:
            diff = self.pre_speed_coord - self.speed_coord
            self.pre_speed_coord = self.speed_coord
            vel = np.abs(np.sum(np.asarray(self.vels)))
            self.vels=[]
            if diff[0] > 0.1:
                self.heading=diff[1]
                speed=diff[0]
            else:
                speed=0.0
            print vel, speed, speed*3.600#self.heading, vel
        else:
            self.pre_speed_coord = self.speed_coord



    def average_coords(self):
        norths = [x.northing for x in self.mti_coords]
        easts = [x.easting for x in self.mti_coords]
        
        meast = np.average(np.asarray(easts))
        mnorth = np.average(np.asarray(norths))
        
        a = utm.to_latlon(meast, mnorth, self.mti_coord.zone_number, self.mti_coord.zone_letter)
        #print meast, mnorth
        self.mti_coords=[]
        return MapCoords(a[0],a[1])
        

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
        if self.dgps_coord:
            self.pre_dgps_coord=self.dgps_coord
        if not np.isnan(data.latitude):
            self.dgps_coord = MapCoords(data.latitude,data.longitude)
            self.gps_canvas.draw_coordinate(self.dgps_coord,'red',size=2, thickness=1, alpha=255)
#            self.draw_coordinate(data.latitude, data.longitude,'red',size=2, thickness=1, alpha=255)
            


    def mti_callback(self, data):
        if self.mti_coord:
            if self.pre_mti_coord:
                diff = self.mti_coord - self.pre_mti_coord
                if diff[0] > 0.5 :
                    self.pre_mti_coord=self.mti_coord
            else:
                self.pre_mti_coord=self.mti_coord

                    
        if not np.isnan(data.latitude):
            self.mti_coord = MapCoords(data.latitude,data.longitude)
            self.mti_coords.append(self.mti_coord)
            self.gps_canvas.draw_coordinate(self.mti_coord,'white',size=2, thickness=1, alpha=255)
            if self.pre_mti_coord:
                diff = self.mti_coord - self.pre_mti_coord
#                if diff[0]>0.5:
#                    print diff
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
    rospy.init_node('view_nav_data')
    SimpleDataVisualiser(18, 640, 10)
