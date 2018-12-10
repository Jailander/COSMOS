#!/usr/bin/env python

import sys
import rospy
import numpy as np

import std_msgs.msg
import sensor_msgs.msg
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Pose
from std_srvs.srv import Trigger


from kriging_exploration.map_coords import MapCoords

class take_scan(object):

    def __init__(self) :
        self.take_scan=False
        self.send_home=False
        self.move_n_metres=False
        self.first_time=True
        rospy.Subscriber('/joy', sensor_msgs.msg.Joy, self.JoyCallback)      
#        self.cmd_pub = rospy.Publisher('/cosmos_scan', std_msgs.msg.String, latch=False, queue_size=1)
        rospy.Subscriber("/navsat_fix", NavSatFix, self.gps_callback)
        rospy.Subscriber("/fix", NavSatFix, self.mti_callback)
        #self.info_pub = rospy.Publisher('/penetrometer_scan', std_msgs.msg.String, latch=False, queue_size=1)        
        tim1 = rospy.Timer(rospy.Duration(0.5), self.speed_calc_timer)
        rospy.loginfo("All Done ...")
        rospy.spin()
        tim1.shutdown()

    def gps_callback(self, data):
        if not np.isnan(data.latitude):
            self.gps_coord = MapCoords(data.latitude,data.longitude)
            
#            self.draw_coordinate(data.latitude, data.longitude,'red',size=2, thickness=1, alpha=255)

    def mti_callback(self, data):
        if not np.isnan(data.latitude):
            self.mti_coord = MapCoords(data.latitude,data.longitude)
#            self.draw_coordinate(data.latitude, data.longitude,'white',size=2, thickness=1, alpha=255)


    def speed_calc_timer(self, event):
        diff1=self.compare-self.gps_coord
        diff2=self.compare2-self.mti_coord
        
        print diff1[0], diff2[0]

    def JoyCallback(self, msg) :
        if not self.first_time:
            if msg.buttons[0] and not self.pre_msg.buttons[0]:
                self.compare=self.gps_coord
                self.compare2=self.mti_coord
        self.first_time = False
        self.pre_msg=msg


if __name__ == '__main__':
    rospy.init_node('take_scan')
    server = take_scan()