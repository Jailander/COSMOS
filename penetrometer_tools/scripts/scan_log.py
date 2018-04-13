#!/usr/bin/env python

# license removed for brevity

import time
import yaml
import rospy

import matplotlib.pyplot as plt
import numpy as np

from std_msgs.msg import Float32
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix
from auto_soil_probe.msg import Controller
from auto_soil_probe.msg import ProbeStatus





class get_penetromter_data(object):

    def __init__(self) :
        self.scanning = False
        self.grounded = False
        self.default_force = 0.0
        self.data_out={}
        
        rospy.on_shutdown(self.on_shutdown)
        rospy.Subscriber("/auto_soil_probe/data", Controller, self.data_callback)
        rospy.Subscriber("/penetrometer/force", Float32, self.force_data_callback)
        #rospy.Subscriber("/navsat_fix", NavSatFix, self.gps_data_callback)
        rospy.Subscriber("/fix", NavSatFix, self.gps_data_callback)
        #rospy.Subscriber("/auto_soil_probe/asp_cmd", String, self.cmd_data_callback)
        rospy.Subscriber("/penetrometer_scan", String, self.cmd_data_callback)
        rospy.Subscriber("/auto_soil_probe/probe", ProbeStatus, self.probe_data_callback)
        rospy.spin()


    def probe_data_callback(self, msg):
        if self.scanning:
            if msg.completed:
                self.data_out['success']=True
                self.create_file()
        

    def cmd_data_callback(self, msg):
        if msg.data == 'Reading':
            print "Scanning"
            self.scanning = True
            self.data_out['stamp'] = self.gps_pos.header.stamp.secs
            self.data_out['fix']={}
            self.data_out['fix']['lat'] = self.gps_pos.latitude
            self.data_out['fix']['lon'] = self.gps_pos.longitude
            self.data_out['fix']['alt'] = self.gps_pos.altitude
            self.data_out['data'] = []
            self.scanning = True
        if msg.data == 'Cancel':
            print "Cancel"
            if self.scanning:
                self.data_out['success']=False
                self.create_file()            
        #if msg.data == 'c':
            #self.scanning = False

    def gps_data_callback(self, msg):
        self.gps_pos = msg

    def force_data_callback(self, msg):
        if self.scanning:
            data={}
            data["z"]=self.zpos
            data["force"] = msg.data
            self.data_out["data"].append(data)


    def data_callback(self, msg):
        self.zpos = msg.position.z
#        if not self.grounded:
#            if msg.position.z >= 480.0:
#                self.grounded=True
#        else:
#            if msg.position.z <= 5.0:
#                self.grounded=False
#                if self.scanning:
#                    self.create_file()

    def create_file(self):                    
        self.scanning = False
        print "STOP"

        outfile = str(self.data_out['stamp'])+'.yaml'            
        yml = yaml.safe_dump(self.data_out, default_flow_style=False)
        fh = open(outfile, "w")
        s_output = str(yml)
        #print s_output
        fh.write(s_output)
        fh.close
            


    
    def on_shutdown(self):
        print "BYE"
    
if __name__ == '__main__':
    rospy.init_node('data_logger')
    server = get_penetromter_data()
