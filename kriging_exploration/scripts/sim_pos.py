#!/usr/bin/env python

import math
import rospy
from std_msgs.msg import Float32
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from kriging_exploration.map_coords import MapCoords
#import time
from sensor_msgs.msg import NavSatFix
from cosmos_msgs.srv import TeleportRobot

class SImPos(object):

    def __init__(self, lat, lon, northang):
        self.gps_coord = MapCoords(lat,lon)
        self.northang = northang
        self.cmd = Twist()
        self.two_pi = math.pi * 2.0

        self.timer= rospy.Timer(rospy.Duration(0.5), self.time_callback)
        rospy.Subscriber("/cmd_vel", Twist, self.callback)
        self.pub = rospy.Publisher("/navsat_fix", NavSatFix)
        self.ang_pub = rospy.Publisher("/septentrio/heading", Float32)
        rospy.Service('/teleport_robot', TeleportRobot, self.teleport_cb)
        
        rospy.spin()
        self.timer.shutdown()


    def teleport_cb(self, req):
        print req
        self.gps_coord = MapCoords(req.coordinates.latitude + 0.000001,req.coordinates.longitude + 0.000001)
        self.precord = self.gps_coord
        rospy.sleep(2.0)
        return True

    def callback(self, data):
        #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data)
        self.cmd = data
    
    def time_callback(self, event):
        self.precord = self.gps_coord
        self.northang = self.northang + (self.cmd.angular.z)
        
        if self.northang > self.two_pi:
            self.northang = self.northang - self.two_pi
        if self.northang < -self.two_pi:
            self.northang = self.northang + self.two_pi
        northing_dif = (self.cmd.linear.x*0.5) * math.cos(self.northang)
        easting_dif =  (self.cmd.linear.x*0.5) * math.sin(self.northang)
        if northing_dif != 0.0 or easting_dif !=0:
            self.gps_coord = self.gps_coord._get_rel_point(easting_dif, northing_dif)
        
        #print self.cmd.linear.x, self.cmd.angular.z
        #print self.northang, northing_dif, easting_dif #self.gps_coord.northing, self.gps_coord.easting
       # print self.gps_coord
        fix = NavSatFix()
        fix.latitude = self.gps_coord.lat
        fix.longitude = self.gps_coord.lon
        self.pub.publish(fix)
        self.ang_pub.publish(self.northang)


if __name__ == '__main__':
    rospy.init_node('sim_pos')
#    SImPos(53.262347, -0.527716, 0.0) #COSMOS
#    SImPos(53.267309, -0.532824, 0.0) #Football
    SImPos(53.138604, 0.004182, 0.0) #Airfield
    
#    SImPos(39.432305, 22.79269, 0.0) #Greece

    


