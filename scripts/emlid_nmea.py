#!/usr/bin/env python3
# connect to the TCP server run by emlid rover and publish gps messages as output
import sys
import rospy
import socket
import time
import numpy as np
from sensor_msgs.msg import NavSatFix
from nmea_msgs.msg import Sentence 

import roslib


DEFAULT_CONFIG = {
    'TCP_IP': '192.168.1.177',  
    'TCP_PORT': 9001,           
    'BUFFER_SIZE': 1024,
}

class EmlidReach:

    def __init__(self):
        # Generate msgs
        self.nmea = Sentence()
        self.flag_new_data = False
        
        # Config
        self.get_config()
        self.TCP_IP = self.config['TCP_IP']
        self.TCP_PORT = self.config['TCP_PORT']
        self.BUFFER_SIZE = self.config['BUFFER_SIZE']
        self.socket_listen = None


    def get_config(self):
        # set parameters
        self.config = DEFAULT_CONFIG.copy()
        param_config = rospy.get_param('~emild_config', {})
        self.config.update(param_config)
        rospy.loginfo('[%s]: emlid config: %s', name, self.config)

        

    def init_sockets(self):
        """Initialize sockets for communication with EMLID reach"""

        # try creating a socket for communication
        try:
            self.socket_listen = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        except socket.error, msg:
            rospy.logerr('Failed to create socket. Error code: '+ str(msg[0]) + ' Msg : '+str(msg[1]))
            sys.exit()
        rospy.loginfo('Socket created')

        # try connecting to the desired IP and PORT
        try:
            self.socket_listen.connect((self.TCP_IP, self.TCP_PORT))
        except socket.error, msg:
            rospy.logerr('Failed to connect to address. Error code: '+ str(msg[0]) + ' Msg : '+str(msg[1]))
            sys.exit()
        rospy.loginfo('Socket connected')

    def close_socket(self):
        """Closes the socket connection"""
        self.socket_listen.close()

    def handle_gps_msg(self, data):
        """"Parses the data to ROS nmea sentences

            Args:
                data : raw input in string format from the TCP server on EMLID Rover
        """
        self.nmea.sentence = data
        rospy.loginfo('[%s]: nmea msg: %s', name, data)


    def listener(self):
        """Creates a python node, to publish messages from EMLID Rover as ROS messages"""
        rospy.init_node('emlid_reach_rtk_nmea', anonymous=False)

        pub = rospy.Publisher('emlid_nmea', Sentence, queue_size=100)
        rate = rospy.Rate(50)

        # try to establish connection with the TCP server of emlid
        self.init_sockets()

        # if connection is successful begin collecting data and parsing them
        while not rospy.is_shutdown():

            # try getting data on the channel
            try:

                data = self.socket_listen.recv(self.BUFFER_SIZE)
                if data:
                    self.last_data_tstamp = time.time()
                    self.handle_gps_msg(data)
                else:
                    # if data is not there wait for some time then timeout
                    self.timeout_counter = time.time() - self.last_data_tstamp
                    if self.timeout_counter > self.timeout_threshold:
                        rospy.loginfo('Connection timed out. Closing')
                        self.socket_listen.close()
                        sys.exit()

            except socket.error as (code, msg):
                self.socket_listen.close()
                rospy.loginfo('Exiting with MSG: %s' % (msg))
                sys.exit()

            if self.flag_new_data is True:
                self.flag_new_data = False
                pub.publish(self.nmea)
                pub_freq.tick()

            rate.sleep()

if __name__ == '__main__':
    name = rospy.get_name()

    p = EmlidReach()
    try:
        p.listener()
    except rospy.ROSInterruptException:
        p.close_socket()
        rospy.loginfo('Exiting...')
