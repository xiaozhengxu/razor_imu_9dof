#!/usr/bin/env python

import socket
import select
import time

class IMU_comm:
    def __init__(self):
        self.host = '192.168.17.201'
        self.port = 7778

        self.UDP_IP = "0.0.0.0" #any interface that tries to connect on the laptop  

        self.addr = (self.host, self.port)

        self.imu_sock = self.setup_TCP_sock()
        print "TCP socket setup"
        self.udp_sock = self.setup_UDP_sock()
        print "UDP socket setup"


    def start(self):
        self.imu_sock.send('#o0' + '\n')#stop
        time.sleep(2)
        self.imu_sock.send('#o1' + '\n') #start data stream
        time.sleep(2)
        self.imu_sock.send('#o1' + '\n') #start data stream
        print "Sent start command"

    def setup_UDP_sock(self):
        udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) # UDP
        udp_sock.bind((self.UDP_IP, self.port))
        return udp_sock

    def setup_TCP_sock(self):
        imu_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        #imu_sock = socket.socketopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
        #imu_sock = socket.socketopt(socket.SOL_TCP, socket.TCP_KEEPIDLE, 30)
        #imu_sock = socket.socketopt(socket.SOL_TCP, socket.TCP_KEEPINTVL, 15)

        try:
            imu_sock.connect((self.addr))
        except socket.error, ex:
            print ex

        print "Waiting 5 seconds for imu/raspi to setup"
        time.sleep(5)

        return imu_sock

    def recv(self):
        line = self.udp_sock.recv(4096)
        print line

if __name__ == '__main__':
    imu_comm = IMU_comm()
    imu_comm.start()
    print "Listening..."
    while True:
        imu_comm.recv()