#! /usr/bin/env python

import numpy as np
import rospy
import math

import argparse

import actionlib
from sensor_msgs.msg import NavSatFix
import open_nav.msg

from geometry_msgs.msg import Twist

from cosmos_msgs.srv import TeleportRobot
from kriging_exploration.map_coords import MapCoords

class opennavserver(object):

    _feedback = open_nav.msg.OpenNavActionFeedback()
    _result   = open_nav.msg.OpenNavResult()

    def __init__(self, name, teleport=False):
        self.cancelled = False
        self._action_name = name
        self.first_fix = True
        self.teleport = teleport
        self.maxxvel = 0.6
        self.maxangvel = 0.1
        
        rospy.on_shutdown(self._shutdown)
        
        #self.brand_image_path = rospy.get_param("~brand_image_path",'/tmp/Tweeter_branding.png')
        rospy.loginfo("Creating action servers.")
        print self._action_name
        self._as = actionlib.SimpleActionServer(self._action_name, open_nav.msg.OpenNavAction, execute_cb = self.executeCallback, auto_start = False)
        self._as.register_preempt_callback(self.preemptCallback)

        rospy.Subscriber("/rtk_fix", NavSatFix, self.gps_callback)

        rospy.loginfo(" ...starting")
        self._as.start()
        rospy.loginfo(" ...done")

        rospy.loginfo("Ready ...")

        self.pub = rospy.Publisher('/cmd_vel', Twist)

        rospy.spin()


    def gps_callback(self, data):
        if not np.isnan(data.latitude):
            self.gps_coord = MapCoords(data.latitude,data.longitude)
            if not self.first_fix:
                ang = self.last_coord - self.gps_coord
                self.ang = math.radians(ang[1])
            self.first_fix = False
            self.last_coord=self.gps_coord
            
            #self.refresh()
        
    def executeCallback(self, goal):
        rospy.loginfo("Navigating...")

        print goal

        goal_coord = MapCoords(goal.coords.latitude,goal.coords.longitude)

        print "RESULT"
        #print self.gps_coord - goal_coord

        result = self.navigate(goal_coord)
            
        if not self.cancelled :
            print result
            self._result.success = result
            self._as.set_succeeded(self._result)
                


    def navigate(self, goal_coord):
        dist, dang = self.gps_coord - goal_coord
        dd=round(dist,2)
        de=round(goal_coord.easting-self.gps_coord.easting,2)
        dn=round(goal_coord.northing-self.gps_coord.northing,2)
        print 'D: ', dd, 'E: ', de, 'N: ', dn, 'w: ', dang 

        while dist>=0.5 and not self.cancelled:
            dist, dang = self.gps_coord - goal_coord
            dd=round(dist,2)
            de=round(goal_coord.easting-self.gps_coord.easting,2)
            dn=round(goal_coord.northing-self.gps_coord.northing,2)
            print 'D: ', dd, 'E: ', de, 'N: ', dn, 'w: ', dang 
            rospy.sleep(0.2)

        if not self.cancelled:
            return True
        else:
            return False


    def preemptCallback(self):
        self.cancelled = True
        self._result.success = False
        self._as.set_preempted(self._result)

    def _shutdown(self):
        self.cancelled = True


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("--teleport", type=bool, default=False, help="activate teleport mode (mainly for simulation)")
    args = parser.parse_args()    
    
    rospy.init_node('open_nav')
    server = opennavserver(rospy.get_name(), teleport=args.teleport)