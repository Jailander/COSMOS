#! /usr/bin/env python

import numpy as np
import rospy
import math

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

        rospy.Subscriber("/navsat_fix", NavSatFix, self.gps_callback)

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

        if not self.teleport:
            result = self.navigate(goal_coord)
            
            if not self.cancelled :
                print result
                self._result.success = result
                self._as.set_succeeded(self._result)
        else:
            rospy.wait_for_service('/teleport_robot')
            try:
                print 
                teleport = rospy.ServiceProxy('/teleport_robot', TeleportRobot)
                result = teleport(goal.coords)
                print result
                rospy.sleep(0.5)
                self._result.success = True#result.success
                self._as.set_succeeded(self._result)
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e
                result = False
                self._result.success = result
                self._as.set_succeeded(self._result)
                


    def navigate(self, goal_coord):
        cmd = Twist()
        dist, dang = self.gps_coord - goal_coord
        dang = self.ang - math.radians(dang)
        while dist>=1.0 and not self.cancelled:
            velx=dist*self.maxxvel
            if velx >= self.maxxvel:
                velx=self.maxxvel
            vela=dang*self.maxangvel
            if abs(vela) >= self.maxangvel:
                if vela > 0:
                    vela = self.maxangvel
                else:
                    vela = -self.maxangvel
            cmd.linear.x=velx
            cmd.angular.z=vela
            self.pub.publish(cmd)
            rospy.sleep(0.5)
            dist, d_ang = self.gps_coord - goal_coord
            d_ang = math.radians(d_ang)
            #dang = self.ang - d_ang
            dang = math.atan2(math.sin(self.ang - d_ang), math.cos(d_ang -self.ang))
            print dist, math.degrees(self.ang), math.degrees(d_ang), dang, velx, vela

        cmd.linear.x=0.0
        cmd.angular.z=0.0
        self.pub.publish(cmd)

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
        cmd = Twist()
        cmd.linear.x=0.0
        cmd.angular.z=0.0
        self.pub.publish(cmd)

if __name__ == '__main__':
    rospy.init_node('open_nav')
    server = opennavserver(rospy.get_name(), teleport=False)