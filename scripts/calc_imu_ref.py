#!/usr/bin/env python
import rospy
import sys
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion, Vector3, AccelStamped
from math import sqrt
import numpy
import os

class sub:
    def __init__(self, imu_name):
        rospy.init_node('wit_grabber')
        self.callback_count = 0
        self.quatAdd = [0.0, 0.0, 0.0, 0.0]

        topic_name = '/' + imu_name + '/imu_stream'
        
        rospy.Subscriber(topic_name, Imu, self.callback, queue_size = 1)

        rospy.spin()

        quatAve = list()
        for num in self.quatAdd:
            quatAve.append(num/self.callback_count)
        
        file_dir = os.path.abspath(os.path.dirname(os.path.dirname(__file__)))
        file_name = file_dir + '/imu_ref/' + imu_name + '.ref'
        print ('saving reference file in ' + file_name)
        with open(file_name, 'w') as f:
            for num in quatAve:
                line = str(num) + '\n'
                f.write(line)
        
            
        #print quatAve        
        

    def callback(self, imu_data):
        quat = imu_data.orientation;
        self.quatAdd[0] += quat.x
        self.quatAdd[1] += quat.y
        self.quatAdd[2] += quat.z
        self.quatAdd[3] += quat.w
        self.callback_count += 1
        #print self.quatAdd
        print [quat.x, quat.y, quat.z, quat.w]


if __name__ == '__main__':

    #print os.getcwd()

    if len(sys.argv) >=2:
        imu_name = sys.argv[1]
    else:
        print "IMU name required!"
        sys.exit()
    imu_sub = sub(imu_name)


