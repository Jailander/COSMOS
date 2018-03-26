#!/usr/bin/env python

# license removed for brevity

import rospy
from std_msgs.msg import Float32
from std_msgs.msg import String
from std_srvs.srv import Trigger
import serial
import time


class force_sensor(object):
    def __init__(self):
        pub = rospy.Publisher('/penetrometer/force', Float32, queue_size=10)
        serv = rospy.Service('/reset_force_sensor', Trigger, self.reset_callback)
        print "opening serial port"
        self.ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=0, parity=serial.PARITY_NONE)
        self.ser.write('\r')
        rospy.sleep(0.1)
        
        bytesToRead = self.ser.inWaiting()
        sdata = self.ser.read(bytesToRead)
    
        print sdata
        print "reseting force sensor"
    
        self.ser.write('CT0\r')
        rospy.sleep(0.1)
    
        bytesToRead = self.ser.inWaiting()
        sdata = self.ser.read(bytesToRead)
    
        print sdata
        print "reseting force sensor"
    
        self.inreset=False
        self.ser.write('O0W0\r')
        rate = rospy.Rate(20) # 10hz
        rospy.sleep(0.01)
        while not rospy.is_shutdown():
            #hello_str = ser.readline()
            if not self.inreset:
                bytesToRead = self.ser.inWaiting()
                sdata = self.ser.read(bytesToRead)
                datao = sdata.split('\r\n')
        #        print sdata, bytesToRead
                if len(datao)>2:
        #            print '--' 
        #            print datao[1]
        #            print '::'
                    rospy.sleep(0.05)
            #        rospy.loginfo(hello_str)
                    val = (float(datao[1])*0.453592)/1000.0
                    pub.publish(val)
                else:
                    print "WARNING"
                    print sdata
            rate.sleep()
        
        self.ser.write('\n')
        rospy.sleep(0.1)
        bytesToRead = self.ser.inWaiting()
        sdata = self.ser.read(bytesToRead)   
        #ser.flush()
        self.ser.flushInput()
        self.ser.flushOutput()
        rospy.sleep(0.1)
        self.ser.close()
        print "All done"

    def reset_callback(self,req):
        self.inreset=True
        self.ser.write('CT0\r')
        rospy.sleep(0.1)
        self.ser.write('O0W0\r')
        rospy.sleep(0.01)        
        self.inreset=False
        return True, "force sensor reset"

if __name__ == '__main__':
    rospy.init_node('force_sensor_interface')   
    force_sensor()
