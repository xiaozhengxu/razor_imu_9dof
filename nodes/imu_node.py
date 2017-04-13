#!/usr/bin/env python

# Copyright (c) 2012, Tang Tiong Yew
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#    * Neither the name of the Willow Garage, Inc. nor the names of its
#      contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

#This script sends commands to the socket to raspi, recieves data from it and 
#process the data into a ROS IMU message to publish to a imu topic

import rospy
import serial
import string
import math
import sys

import socket

#from time import time
from sensor_msgs.msg import Imu
from tf.transformations import quaternion_from_euler
from dynamic_reconfigure.server import Server
from razor_imu_9dof.cfg import imuConfig
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

degrees2rad = math.pi/180.0
imu_yaw_calibration = 0.0


class imu_node():

    def __init__(self):
        self.host = rospy.get_param('~port', '192.168.17.201')

        UDP_IP = "0.0.0.0" #any interface that tries to connect on the laptop  
        # UDP_IP = '' #symbolic for all interfaces
        UDP_PORT = 7778

        self.addr = (UDP_IP,UDP_PORT)
        #udp to get sensor command 
        self.imu_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) # UDP
        self.imu_sock.bind((UDP_IP, UDP_PORT))
        # self.imu_sock.listen(1)
        # conn, addr = self.imu_sock.accept()

        # self.imu_sock.connect((self.host,UDP_PORT))
        self.imu_sock.settimeout(.02)
        print "CONNECTED to imu!"

        rospy.init_node("razor_node")
        #We only care about the most recent measurement, i.e. queue_size=1
        self.pub = rospy.Publisher('imu', Imu, queue_size=1)
        self.srv = Server(imuConfig, self.reconfig_callback)  # define dynamic_reconfigure callback
        self.diag_pub = rospy.Publisher('diagnostics', DiagnosticArray, queue_size=1)
        self.diag_pub_time = rospy.get_time();

        self.imuMsg = Imu()

        # Orientation covariance estimation:
        # Observed orientation noise: 0.3 degrees in x, y, 0.6 degrees in z
        # Magnetometer linearity: 0.1% of full scale (+/- 2 gauss) => 4 milligauss
        # Earth's magnetic field strength is ~0.5 gauss, so magnetometer nonlinearity could
        # cause ~0.8% yaw error (4mgauss/0.5 gauss = 0.008) => 2.8 degrees, or 0.050 radians
        # i.e. variance in yaw: 0.0025
        # Accelerometer non-linearity: 0.2% of 4G => 0.008G. This could cause
        # static roll/pitch error of 0.8%, owing to gravity orientation sensing
        # error => 2.8 degrees, or 0.05 radians. i.e. variance in roll/pitch: 0.0025
        # so set all covariances the same.
        self.imuMsg.orientation_covariance = [
        0.0025 , 0 , 0,
        0, 0.0025, 0,
        0, 0, 0.0025
        ]

        # Angular velocity covariance estimation:
        # Observed gyro noise: 4 counts => 0.28 degrees/sec
        # nonlinearity spec: 0.2% of full scale => 8 degrees/sec = 0.14 rad/sec
        # Choosing the larger (0.14) as std dev, variance = 0.14^2 ~= 0.02
        self.imuMsg.angular_velocity_covariance = [
        0.02, 0 , 0,
        0 , 0.02, 0,
        0 , 0 , 0.02
        ]

        # linear acceleration covariance estimation:
        # observed acceleration noise: 5 counts => 20milli-G's ~= 0.2m/s^2
        # nonliniarity spec: 0.5% of full scale => 0.2m/s^2
        # Choosing 0.2 as std dev, variance = 0.2^2 = 0.04
        self.imuMsg.linear_acceleration_covariance = [
        0.04 , 0 , 0,
        0 , 0.04, 0,
        0 , 0 , 0.04
        ]
        #accelerometer
        self.accel_x_min = rospy.get_param('~accel_x_min', -250.0)
        self.accel_x_max = rospy.get_param('~accel_x_max', 250.0)
        self.accel_y_min = rospy.get_param('~accel_y_min', -250.0)
        self.accel_y_max = rospy.get_param('~accel_y_max', 250.0)
        self.accel_z_min = rospy.get_param('~accel_z_min', -250.0)
        self.accel_z_max = rospy.get_param('~accel_z_max', 250.0)

        # magnetometer
        self.magn_x_min = rospy.get_param('~magn_x_min', -600.0)
        self.magn_x_max = rospy.get_param('~magn_x_max', 600.0)
        self.magn_y_min = rospy.get_param('~magn_y_min', -600.0)
        self.magn_y_max = rospy.get_param('~magn_y_max', 600.0)
        self.magn_z_min = rospy.get_param('~magn_z_min', -600.0)
        self.magn_z_max = rospy.get_param('~magn_z_max', 600.0)
        self.calibration_magn_use_extended = rospy.get_param('~calibration_magn_use_extended', False)
        self.magn_ellipsoid_center = rospy.get_param('~self.magn_ellipsoid_center', [0, 0, 0])
        self.magn_ellipsoid_transform = rospy.get_param('~self.magn_ellipsoid_transform', [[0, 0, 0], [0, 0, 0], [0, 0, 0]])
        self.imu_yaw_calibration = rospy.get_param('~imu_yaw_calibration', 0.0)

        # gyroscope
        self.gyro_average_offset_x = rospy.get_param('~gyro_average_offset_x', 0.0)
        self.gyro_average_offset_y = rospy.get_param('~gyro_average_offset_y', 0.0)
        self.gyro_average_offset_z = rospy.get_param('~gyro_average_offset_z', 0.0)

        #rospy.loginfo("%f %f %f %f %f %f", accel_x_min, accel_x_max, accel_y_min, accel_y_max, accel_z_min, accel_z_max)
        #rospy.loginfo("%f %f %f %f %f %f", magn_x_min, magn_x_max, magn_y_min, magn_y_max, magn_z_min, magn_z_max)
        #rospy.loginfo("%s %s %s", str(calibration_magn_use_extended), str(self.magn_ellipsoid_center), str(self.magn_ellipsoid_transform[0][0]))
        #rospy.loginfo("%f %f %f", gyro_average_offset_x, gyro_average_offset_y, gyro_average_offset_z)


        self.roll=0
        self.pitch=0
        self.yaw=0
        self.seq=0
        self.accel_factor = 9.806 / 256.0    # sensor reports accel as 256.0 = 1G (9.8m/s^2). Convert to m/s^2.
        rospy.loginfo("Giving the razor IMU board 5 seconds to boot...")
        rospy.sleep(5) # Sleep for 5 seconds to wait for the board to boot


        ### configure board ###
        #stop datastream
        #sendto is udp
        self.imu_sock.sendto('#o0' + chr(13),self.addr)

        #TO-DO Maybe flush input here

        #set output mode
        self.imu_sock.sendto('#ox' + chr(13),self.addr) # To start display angle and sensor reading in text

        rospy.loginfo("Writing calibration values to razor IMU board...")

        #set calibration values
        self.imu_sock.sendto('#caxm' + str(self.accel_x_min) + chr(13),self.addr)
        self.imu_sock.sendto('#caxM' + str(self.accel_x_max) + chr(13),self.addr)
        self.imu_sock.sendto('#caym' + str(self.accel_y_min) + chr(13),self.addr)
        self.imu_sock.sendto('#cayM' + str(self.accel_y_max) + chr(13),self.addr)
        self.imu_sock.sendto('#cazm' + str(self.accel_z_min) + chr(13),self.addr)
        self.imu_sock.sendto('#cazM' + str(self.accel_z_max) + chr(13),self.addr)

        #calibration values for magnetometer
        if (not self.calibration_magn_use_extended):
            self.imu_sock.sendto('#cmxm' + str(self.magn_x_min) + chr(13),self.addr)
            self.imu_sock.sendto('#cmxM' + str(self.magn_x_max) + chr(13),self.addr)
            self.imu_sock.sendto('#cmym' + str(self.magn_y_min) + chr(13),self.addr)
            self.imu_sock.sendto('#cmyM' + str(self.magn_y_max) + chr(13),self.addr)
            self.imu_sock.sendto('#cmzm' + str(self.magn_z_min) + chr(13),self.addr)
            self.imu_sock.sendto('#cmzM' + str(self.magn_z_max) + chr(13),self.addr)
        else:
            self.imu_sock.sendto('#ccx' + str(self.magn_ellipsoid_center[0]) + chr(13),self.addr)
            self.imu_sock.sendto('#ccy' + str(self.magn_ellipsoid_center[1]) + chr(13),self.addr)
            self.imu_sock.sendto('#ccz' + str(self.magn_ellipsoid_center[2]) + chr(13),self.addr)
            self.imu_sock.sendto('#ctxX' + str(self.magn_ellipsoid_transform[0][0]) + chr(13),self.addr)
            self.imu_sock.sendto('#ctxY' + str(self.magn_ellipsoid_transform[0][1]) + chr(13),self.addr)
            self.imu_sock.sendto('#ctxZ' + str(self.magn_ellipsoid_transform[0][2]) + chr(13),self.addr)
            self.imu_sock.sendto('#ctyX' + str(self.magn_ellipsoid_transform[1][0]) + chr(13),self.addr)
            self.imu_sock.sendto('#ctyY' + str(self.magn_ellipsoid_transform[1][1]) + chr(13),self.addr)
            self.imu_sock.sendto('#ctyZ' + str(self.magn_ellipsoid_transform[1][2]) + chr(13),self.addr)
            self.imu_sock.sendto('#ctzX' + str(self.magn_ellipsoid_transform[2][0]) + chr(13),self.addr)
            self.imu_sock.sendto('#ctzY' + str(self.magn_ellipsoid_transform[2][1]) + chr(13),self.addr)
            self.imu_sock.sendto('#ctzZ' + str(self.magn_ellipsoid_transform[2][2]) + chr(13),self.addr)

        #Compensating for offset of gyro-drift 
        self.imu_sock.sendto('#cgx' + str(self.gyro_average_offset_x) + chr(13),self.addr)
        self.imu_sock.sendto('#cgy' + str(self.gyro_average_offset_y) + chr(13),self.addr)
        self.imu_sock.sendto('#cgz' + str(self.gyro_average_offset_z) + chr(13),self.addr)

        #print calibration values for verification by user
        self.imu_sock.sendto('#p' + chr(13),self.addr)
        calib_data = self.imu_sock.recv(4096)
        calib_data_print = "Printing set calibration values:\r\n"
        for line in calib_data:
            calib_data_print += line
        rospy.loginfo(calib_data_print)

        #start datastream
        self.imu_sock.sendto('#ox' + chr(13),self.addr)

        #automatic flush - NOT WORKING
        #ser.flushInput()  #discard old input, still in invalid format
        #flush manually, as above command is not working - it breaks the serial connection
        rospy.loginfo("Flushing first 200 IMU entries...")
        for x in range(0, 10):
            line = self.imu_sock.recv(4096)
        rospy.loginfo("Publishing IMU data...")
        #f = open("raw_imu_data.log", 'w')


        # Callback for dynamic reconfigure requests
    def reconfig_callback(self,config, level):
        global imu_yaw_calibration
        rospy.loginfo("""Reconfigure request for yaw_calibration: %d""" %(config['yaw_calibration']))
        #if imu_yaw_calibration != config('yaw_calibration'):
        imu_yaw_calibration = config['yaw_calibration']
        rospy.loginfo("Set imu_yaw_calibration to %d" % (imu_yaw_calibration))
        return config

    def run(self):
        while not rospy.is_shutdown():
            line = self.imu_sock.recv(4096)
            line = line.replace("#YPRAG=","")   # Delete "#YPRAG="
            #f.write(line)                     # Write to the output log file
            words = string.split(line,",")    # Fields split
            if len(words) > 2:
                #in AHRS firmware z axis points down, in ROS z axis points up (see REP 103)
                yaw_deg = -float(words[0])
                yaw_deg = yaw_deg + imu_yaw_calibration
                if yaw_deg > 180.0:
                    yaw_deg = yaw_deg - 360.0
                if yaw_deg < -180.0:
                    yaw_deg = yaw_deg + 360.0
                self.yaw = yaw_deg*degrees2rad
                #in AHRS firmware y axis points right, in ROS y axis points left (see REP 103)
                self.pitch = -float(words[1])*degrees2rad
                self.roll = float(words[2])*degrees2rad

                # Publish message
                # AHRS firmware accelerations are negated
                # This means y and z are correct for ROS, but x needs reversing
                self.imuMsg.linear_acceleration.x = -float(words[3]) * self.accel_factor
                self.imuMsg.linear_acceleration.y = float(words[4]) * self.accel_factor
                self.imuMsg.linear_acceleration.z = float(words[5]) * self.accel_factor

                self.imuMsg.angular_velocity.x = float(words[6])
                #in AHRS firmware y axis points right, in ROS y axis points left (see REP 103)
                self.imuMsg.angular_velocity.y = -float(words[7])
                #in AHRS firmware z axis points down, in ROS z axis points up (see REP 103) 
                self.imuMsg.angular_velocity.z = -float(words[8])

                q = quaternion_from_euler(self.roll,self.pitch,self.yaw)
                self.imuMsg.orientation.x = q[0]
                self.imuMsg.orientation.y = q[1]
                self.imuMsg.orientation.z = q[2]
                self.imuMsg.orientation.w = q[3]
                self.imuMsg.header.stamp= rospy.Time.now()
                self.imuMsg.header.frame_id = 'base_imu_link'
                self.imuMsg.header.seq = self.seq
                self.seq = self.seq + 1
                pub.publish(self.imuMsg)

            if (self.diag_pub_time < rospy.get_time()) :
                self.diag_pub_time += 1
                diag_arr = DiagnosticArray()
                diag_arr.header.stamp = rospy.get_rostime()
                diag_arr.header.frame_id = '1'
                diag_msg = DiagnosticStatus()
                diag_msg.name = 'Razor_Imu'
                diag_msg.level = DiagnosticStatus.OK
                diag_msg.message = 'Received AHRS measurement'
                diag_msg.values.append(KeyValue('roll (deg)',
                                        str(self.roll*(180.0/math.pi))))
                diag_msg.values.append(KeyValue('pitch (deg)',
                                        str(self.pitch*(180.0/math.pi))))
                diag_msg.values.append(KeyValue('yaw (deg)',
                                        str(self.yaw*(180.0/math.pi))))
                diag_msg.values.append(KeyValue('sequence number', str(self.seq)))
                diag_arr.status.append(diag_msg)
                self.diag_pub.publish(diag_arr)
            

if __name__ == '__main__':
    imu_node = imu_node()
    imu_node.run()
