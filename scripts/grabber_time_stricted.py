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
import os

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
        

        self.ref_mode = True
        imu_name = rospy.get_name()
        ref_file_dir = os.path.abspath(os.path.dirname(os.path.dirname(__file__)))
        file_name = ref_file_dir + '/imu_ref' + imu_name + '.ref'
        #print file_name
        self.ori_ref = list()
        with open(file_name) as f:
            for line in f:
                num_str = line.rstrip()
                self.ori_ref.append(float(num_str))
        #self.ori_ref[3] = -self.ori_ref[3]
        #print self.ori_ref


        self.port = serial.Serial(port_name, 9600)
        self.firstRead = True

        self.frameTime = rospy.Time.now()
        
        self.Acc = Vector3(0.0, 0.0, 0.0)
        self.Quat = Quaternion(0.0, 0.0, 0.0 ,1.0)
        #self.normQuat = Quaternion(0.0, 0.0, 0.0 ,1.0)
        self.refQuat = Quaternion(0.0, 0.0, 0.0 ,1.0)
        self.Gyro = Vector3(0.0, 0.0, 0.0)
        self.AccCart = Vector3(0.0, 0.0, 0.0)

        self.imuPub = rospy.Publisher('~imu_stream', Imu, queue_size=1)
        #self.accPub = rospy.Publisher('~proc_acc', AccelStamped, queue_size=1)
        self.imu_name = rospy.get_param('~imu_name')

        self.br = tf.TransformBroadcaster()


    def ReadFrame(self):
        start = time.clock()
        if self.firstRead == True:
            self.port.flushInput()
            self.firstRead = False
        
        while True:
            head = self.port.read()
            if ord(head) == 85 :
                data_case = self.port.read()
                if ord(data_case) == 80: #case Time
                    data = self.port.read(9)
                    hour = ord(data[3])
                    minute = ord(data[4])
                    sec = ord(data[5])
                    ms = ord(data[7]) * 256 + ord(data[6])
                    SUM = 85 + 80
                    for i in range(0, 8):
                        SUM += ord(data[i])
                    res = SUM % 256 
                    if not res == ord(data[8]):
                        #print res, ord(data[8])
                        continue
                    #print "Time: ", hour, "h ", minute, "min ", sec, "s ", ms, "ms"
                    self.frameTime = rospy.Time.now()

                    break
                else:
                    continue
        
        #case Accelemeter
        self.port.read()
        data = self.port.read(10)
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
        else :
            print "Acceleration Data Lost"
            
        '''
        #case Gyro
        self.port.read()
        data = self.port.read(10)
        if ord(data[0]) == 83 :
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
        else :
            print "Gyro Data Lost"

        
        #case Magnet
        self.port.read()
        data = self.port.read(10)
        if ord(data[0]) == 84 :
            pass
        else:
            print "Magnet Data Lost"

        '''
        #case Quaternion
        self.port.read()
        data = self.port.read(10)
        if ord(data[0]) == 89 :
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
            '''
            quatNorm = sqrt(self.Quat.x * self.Quat.x +
                            self.Quat.y * self.Quat.y +
                            self.Quat.z * self.Quat.z +
                            self.Quat.w * self.Quat.w)
            self.normQuat.x = self.Quat.x / quatNorm
            self.normQuat.y = self.Quat.y / quatNorm
            self.normQuat.z = self.Quat.z / quatNorm
            self.normQuat.w = self.Quat.w / quatNorm
            '''
            self.normQuat = self.Quat
            quat_origin = [self.normQuat.x, self.normQuat.y, self.normQuat.z, self.normQuat.w]
            if self.ref_mode:
                ref_quat = tf.transformations.quaternion_multiply(
                        tf.transformations.quaternion_conjugate(self.ori_ref),
                        quat_origin)
                self.refQuat = Quaternion(ref_quat[0], ref_quat[1], ref_quat[2], ref_quat[3])
                quat_origin = ref_quat


            self.br.sendTransform((0, 0, 0),
                             quat_origin,
                             rospy.Time.now(),
                             self.imu_name,
                             "human_base")
        else :
            print "Quaternion Data Lost"

        end = time.clock()
        #print 'Running time: ' ,end - start

    def eraseGravity(self):
        if self.ref_mode:
            q1 = [self.refQuat.x, self.refQuat.y, self.refQuat.z, self.refQuat.w]
        else:
            q1 = [self.Quat.x, self.Quat.y, self.Quat.z, self.Quat.w]

        #q1 = list(q1)
        acc = [self.Acc.x, self.Acc.y, self.Acc.z]
        acc = numpy.array(acc, dtype=numpy.float64,copy=True)
        acc_scalar = sqrt(numpy.dot(acc, acc))
        unit_acc = acc/acc_scalar
        q2 = list(unit_acc)
        q2.append(0.0)
        '''        
        acc_no_grav = tf.transformations.quaternion_multiply(
                tf.transformations.quaternion_multiply(q1, q2),
                tf.transformations.quaternion_conjugate(q1)
                )

        acc_no_grav = tf.transformations.quaternion_multiply(
                tf.transformations.quaternion_multiply(self.ori_ref, acc_no_grav),
                tf.transformations.quaternion_conjugate(self.ori_ref)
                )[0:3]
        '''

        acc_no_grav = tf.transformations.quaternion_multiply(
                tf.transformations.quaternion_multiply(q1, q2),
                tf.transformations.quaternion_conjugate(q1)
                )[0:3]
        acc_no_grav *= acc_scalar
        acc_no_grav -= [0.0, 0.0, 9.8]
        
        #for i in range(0, 3):
        #    if acc_no_grav[i] < 0.4 and acc_no_grav[i] > -0.4:
        #        acc_no_grav[i] = 0.0
        
        #PubAcc = Vector3(acc_no_grav[0], acc_no_grav[1], acc_no_grav[2])
        self.AccCart = Vector3(acc_no_grav[0], acc_no_grav[1], acc_no_grav[2])
        #acc_msg = AccelStamped()
        #acc_msg.header.stamp = self.frameTime
        #acc_msg.accel.linear = PubAcc

        #self.accPub.publish(acc_msg)

        #print acc_no_grav


    def publishImuData(self):
        imu_msg = Imu()
        imu_msg.header.stamp = self.frameTime
        #imu_msg.linear_acceleration = self.Acc
        imu_msg.linear_acceleration = self.AccCart
        if self.ref_mode:
            imu_msg.orientation = self.refQuat
        else:
            imu_msg.orientation = self.Quat
        imu_msg.angular_velocity = self.Acc

        #print imu_msg

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
        time.sleep(0.5)
        
        ax_ave = 0
        ay_ave = 0
        az_ave = 0
        
        self.port.flushInput()

        count = 0
        
        while count < 100:
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
                #print ax
            
                temp = ord(data[4]) * 256 + ord(data[3])
                ss = struct.pack('H', temp)
                ay, = struct.unpack('h' , ss)
                ay_ave += ay
                #print ay
            
                temp = ord(data[6]) * 256 + ord(data[5])
                ss = struct.pack('H', temp)
                az, = struct.unpack('h' , ss)
                az_ave += az
                #print az

                count += 1
                #print("count: ", count)
            else:
                continue
        
        ax_e = ax_ave / 100
        ay_e = ay_ave / 100
        az_e = az_ave / 100 - 2048
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

    def setAxis9(self):
        
        cmd = 'ffaa240000'
        cmd_hex = cmd.decode("hex")
        self.port.write(cmd_hex)
        time.sleep(0.1)

        cmd = 'ffaa000000'
        cmd_hex = cmd.decode("hex")
        self.port.write(cmd_hex)
        time.sleep(0.1)

    def setPassingFreq(self):
        
        cmd = 'ffaa030700'
        cmd_hex = cmd.decode("hex")
        self.port.write(cmd_hex)
        time.sleep(0.1)

        cmd = 'ffaa000000'
        cmd_hex = cmd.decode("hex")
        self.port.write(cmd_hex)
        time.sleep(0.1)

    def setPassingTerm(self):
        
        cmd = 'ffaa020302' #Enable Time Quat and Acc
        cmd_hex = cmd.decode("hex")
        self.port.write(cmd_hex)
        time.sleep(0.1)

        cmd = 'ffaa000000'
        cmd_hex = cmd.decode("hex")
        self.port.write(cmd_hex)
        time.sleep(0.1)

if __name__ == '__main__':

    do_calib = False
    rospy.init_node('wit_grabber')
    if len(sys.argv) >=2:
        port = sys.argv[1]
    else:
        print "Port name required!"
        sys.exit()
    if len(sys.argv) >=3:
        do_calib = True

    imu_grabber = Imu_Grabber(port)

    #time.sleep(0.1)
    imu_grabber.setPassingFreq()
    imu_grabber.setPassingTerm()

    #if do_calib:
    #imu_grabber.calibrateAcc()
    #imu_grabber.setZAxistoZero()
    
    rate = rospy.Rate(150)
    
    imu_grabber.ReadFrame()
    while not rospy.is_shutdown():
        imu_grabber.ReadFrame()
        imu_grabber.eraseGravity()
        imu_grabber.publishImuData()
        rate.sleep()
    
