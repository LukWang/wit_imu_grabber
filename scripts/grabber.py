#!/usr/bin/env python
import sys
import serial
import string
import time
import rospy
import struct
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion, Vector3, AccelStamped
import tf
from math import sqrt
import numpy

def hexShow(argv):  
    result = ''  
    hLen = len(argv)  
    for i in xrange(hLen):  
        hvol = ord(argv[i]) 
        hhex = '%02x'%hvol  
        result += hhex+' '  
    print 'hexShow:',result

def NumtoHex(Num):
    temp = struct.pack('h', Num)
    UNum, = struct.unpack('H', temp)
    Hex = hex(UNum)[2:]
    return Hex

class Imu_Grabber:
    def __init__(self, port_name):
        self.port = serial.Serial(port_name, 9600)
        
        self.Acc = Vector3(0.0, 0.0, 0.0)
        self.Quat = Quaternion(0.0, 0.0, 0.0 ,1.0)
        self.normQuat = Quaternion(0.0, 0.0, 0.0 ,1.0)
        self.Gyro = Vector3(0.0, 0.0, 0.0)

        self.imuPub = rospy.Publisher('~imu_stream', Imu, queue_size=1)
        self.accPub = rospy.Publisher('~proc_acc', AccelStamped, queue_size=1)
        self.imu_name = rospy.get_param('~imu_name')

        self.br = tf.TransformBroadcaster()



    def recvData(self):
        while True:
            head = self.port.read()
            if ord(head) == 85 :
                break
        data = self.port.read(10)
        
        #case Accelemeter
        if ord(data[0]) == 81 :
            temp = ord(data[2]) * 256 + ord(data[1])
            ss = struct.pack('H', temp)
            ax, = struct.unpack('h' , ss)
            self.Acc.x = float(ax) / 32768 * 16 * 9.8
            
            temp = ord(data[4]) * 256 + ord(data[3])
            ss = struct.pack('H', temp)
            ay, = struct.unpack('h' , ss)
            self.Acc.y = float(ay) / 32768 * 16 * 9.8
            
            temp = ord(data[6]) * 256 + ord(data[5])
            ss = struct.pack('H', temp)
            az, = struct.unpack('h' , ss)
            self.Acc.z = float(az) / 32768 * 16 * 9.8
            
        
        #case Euler Angle
        elif ord(data[0]) == 83 :
            temp = ord(data[2]) * 256 + ord(data[1])
            ss = struct.pack('H', temp)
            ox, = struct.unpack('h' , ss)
            self.Gyro.x = float(ox) / 32768.0 * 180.0
            
            temp = ord(data[4]) * 256 + ord(data[3])
            ss = struct.pack('H', temp)
            oy, = struct.unpack('h' , ss)
            self.Gyro.y = float(oy) / 32768.0 * 180.0
            
            temp = ord(data[6]) * 256 + ord(data[5])
            ss = struct.pack('H', temp)
            oz, = struct.unpack('h' , ss)
            self.Gyro.z = float(oz) / 32768.0 * 180.0

        #case Quaternion
        elif ord(data[0]) == 89 :
            temp = ord(data[2]) * 256 + ord(data[1])
            ss = struct.pack('H', temp)
            qx, = struct.unpack('h' , ss)
            self.Quat.w = float(qx) / 32768.0
            
            temp = ord(data[4]) * 256 + ord(data[3])
            ss = struct.pack('H', temp)
            qy, = struct.unpack('h' , ss)
            self.Quat.x = float(qy) / 32768.0
            
            temp = ord(data[6]) * 256 + ord(data[5])
            ss = struct.pack('H', temp)
            qz, = struct.unpack('h' , ss)
            self.Quat.y = float(qz) / 32768.0
            
            temp = ord(data[8]) * 256 + ord(data[7])
            ss = struct.pack('H', temp)
            qw, = struct.unpack('h' , ss)
            self.Quat.z = float(qw) / 32768.0

            quatNorm = sqrt(self.Quat.x * self.Quat.x +
                            self.Quat.y * self.Quat.y +
                            self.Quat.z * self.Quat.z +
                            self.Quat.w * self.Quat.w)
            self.normQuat.x = self.Quat.x / quatNorm
            self.normQuat.y = self.Quat.y / quatNorm
            self.normQuat.z = self.Quat.z / quatNorm
            self.normQuat.w = self.Quat.w / quatNorm

            self.br.sendTransform((0, 0, 0),
                             [self.normQuat.x, self.normQuat.y, self.normQuat.z, self.normQuat.w],
                             rospy.Time.now(),
                             self.imu_name,
                             "world")
#        hexShow(data)


    def eraseGravity(self):
        q1 = [self.Quat.x, self.Quat.y, self.Quat.z, self.Quat.w]
        q1 = list(q1)
        acc = [self.Acc.x, self.Acc.y, self.Acc.z]
        acc = numpy.array(acc, dtype=numpy.float64,copy=True)
        acc_scalar = sqrt(numpy.dot(acc, acc))
        unit_acc = acc/acc_scalar
        q2 = list(unit_acc)
        q2.append(0.0)
        acc_no_grav = tf.transformations.quaternion_multiply(
                tf.transformations.quaternion_multiply(q1, q2),
                tf.transformations.quaternion_conjugate(q1)
                )[:3]
        acc_no_grav *= acc_scalar
        acc_no_grav -= [0.0, 0.0, 9.8]
        PubAcc = Vector3(acc_no_grav[0], acc_no_grav[1], acc_no_grav[2])
        acc_msg = AccelStamped()
        acc_msg.header.stamp =rospy.Time.now()
        acc_msg.accel.linear = PubAcc

        self.accPub.publish(acc_msg)

        #print acc_no_grav


    def publishImuData(self):
        imu_msg = Imu()
        imu_msg.header.stamp = rospy.Time.now()
        imu_msg.linear_acceleration = self.Acc
        imu_msg.orientation = self.normQuat
        imu_msg.angular_velocity = self.Gyro

        self.imuPub.publish(imu_msg)

    def calibrateAcc(self):
        print "Calibrating Acc ..."
        cmd = 'ffaa050000'
        cmd_hex = cmd.decode("hex")
        self.port.write(cmd_hex)
        time.sleep(0.1)

        cmd = 'ffaa060000'
        cmd_hex = cmd.decode("hex")
        self.port.write(cmd_hex)
        time.sleep(0.1)
        
        cmd = 'ffaa070000'
        cmd_hex = cmd.decode("hex")
        self.port.write(cmd_hex)
        time.sleep(0.1)
        
        ax_ave = 0
        ay_ave = 0
        az_ave = 0
        
        self.port.flushOutput()

        count = 0
        
        while count < 40:
            while True:
                head = self.port.read()
                if ord(head) == 85 :
                    break
            data = self.port.read(10)
        
            #case Accelemeter
            if ord(data[0]) == 81 :
                temp = ord(data[2]) * 256 + ord(data[1])
                ss = struct.pack('H', temp)
                ax, = struct.unpack('h' , ss)
                ax_ave += ax
            
                temp = ord(data[4]) * 256 + ord(data[3])
                ss = struct.pack('H', temp)
                ay, = struct.unpack('h' , ss)
                ay_ave += ay
            
                temp = ord(data[6]) * 256 + ord(data[5])
                ss = struct.pack('H', temp)
                az, = struct.unpack('h' , ss)
                az_ave += az

                count += 1
            else:
                continue
        
        ax_e = ax_ave / 40
        ay_e = ay_ave / 40
        az_e = az_ave / 40 - 2048
        print "error ax:", ax_e
        print "error ay:", ay_e
        print "error az:", az_e

        ax_e_hex = NumtoHex(ax_e)
        for num in range(0, 4 - len(ax_e_hex)):
            ax_e_hex = '0' + ax_e_hex
        ax_e_hex_inv = ax_e_hex[2:4] + ax_e_hex[0:2]
        cmd = 'ffaa05' + ax_e_hex_inv
        cmd_hex = cmd.decode("hex")
        self.port.write(cmd_hex)
        time.sleep(0.1)

        ay_e_hex = NumtoHex(ay_e)
        for num in range(0, 4 - len(ay_e_hex)):
            ay_e_hex = '0' + ay_e_hex
        ay_e_hex_inv = ay_e_hex[2:4] + ay_e_hex[0:2]
        cmd = 'ffaa06' + ay_e_hex_inv
        cmd_hex = cmd.decode("hex")
        self.port.write(cmd_hex)
        time.sleep(0.1)
            

        az_e_hex = NumtoHex(az_e)
        for num in range(0, 4 - len(az_e_hex)):
            az_e_hex = '0' + az_e_hex
        az_e_hex_inv = az_e_hex[2:4] + az_e_hex[0:2]
        cmd = 'ffaa07' + az_e_hex_inv
        cmd_hex = cmd.decode("hex")
        self.port.write(cmd_hex)
        time.sleep(0.1)
        
        cmd = 'ffaa000000'
        cmd_hex = cmd.decode("hex")
        self.port.write(cmd_hex)
        time.sleep(0.1)

        print 'Calibraion complete!'

    def setZAxistoZero(self):
        
        cmd = 'ffaa240000'
        cmd_hex = cmd.decode("hex")
        self.port.write(cmd_hex)
        time.sleep(0.1)

        #cmd = 'ffaa3f0000'
        #cmd_hex = cmd.decode("hex")
        #self.port.write(cmd_hex)
        #time.sleep(0.1)

        cmd = 'ffaa000000'
        cmd_hex = cmd.decode("hex")
        self.port.write(cmd_hex)
        time.sleep(0.1)

if __name__ == '__main__':


    rospy.init_node('wit_grabber')
    if len(sys.argv) >=2:
                port = sys.argv[1]
    else:
        print "Port name required!"
        sys.exit()

    imu_grabber = Imu_Grabber(port)

    time.sleep(1)

    #imu_grabber.calibrateAcc()
    imu_grabber.setZAxistoZero()

    rate = rospy.Rate(500)
    while not rospy.is_shutdown():
        imu_grabber.recvData()
        imu_grabber.publishImuData()
        imu_grabber.eraseGravity()
        rate.sleep()
