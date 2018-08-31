#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from std_msgs.msg import Float32
import serial


class SeptentrioReader(object):

    def __init__(self):
        self.truenorth_cal=0.0
        ser = serial.Serial('/dev/ttyACM0', 115000)
        self.pub = rospy.Publisher('/septentrio/string', String, queue_size=10)
        self.ang_pub = rospy.Publisher('/septentrio/heading', Float32, queue_size=0)
        rospy.init_node('talker', anonymous=True)
        rate = rospy.Rate(10) # 10hz
        print("HERE")
        while not rospy.is_shutdown():
            serial_line = ser.readline()
            self.decode_string(serial_line)
            #print("-----")
            rate.sleep()        

    def decode_string(self, serial_line):
        if serial_line.startswith('$PSSN,HRP'):
            print serial_line
            data=serial_line.split(',')
            try:
                val = float(data[4])
            except ValueError:
                val=-1.0
            self.ang_val = float(val)+self.truenorth_cal
            self.pub.publish(data[4])
            self.ang_pub.publish(val)

if __name__ == '__main__':
    SeptentrioReader()
    