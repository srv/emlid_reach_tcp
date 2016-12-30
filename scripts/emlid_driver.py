#!/usr/bin/env python
# connect to the TCP server run by emlid rover and publish gps messages as output
import sys
import rospy
import socket
import errno
import time
import numpy as np
from sensor_msgs.msg import NavSatFix
import parse_RTK_msgs as parser


class EmlidReach:

    def __init__(self):
        self.gps_data = NavSatFix()
        self.flag_new_data = False
        
        self.TCP_IP = '192.168.2.15'
        self.TCP_PORT = 5001
        self.BUFFER_SIZE = 1024

        self.timeout_counter = 0
        self.timeout_threshold = 5.0
        self.last_data_tstamp = time.time()

        self.socket_listen = None

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
        """"Parses the data to ROS NavSatFix messages

            Args:
                data : raw input in string format from the TCP server on EMLID Rover
        """
        res = parser.parse_llh_msgs(data)
        if res is not None:
            self.flag_new_data = True

            # store the gps data from emlid into the Navsat fix msg from ROS
            self.gps_data.header.frame_id = 'kf'
            # Note this time contains the system time, for getting gps time use timereference message under sensor_msgs
            self.gps_data.header.stamp = rospy.get_rostime()
            self.gps_data.latitude = res['lat']
            self.gps_data.longitude = res['lon']
            self.gps_data.altitude = res['alt']
            self.gps_data.position_covariance[0] = res['sde']**2
            self.gps_data.position_covariance[1] = res['sdne'] * np.fabs(res['sdne'])
            self.gps_data.position_covariance[2] = res['sdeu'] * np.fabs(res['sdeu'])
            self.gps_data.position_covariance[3] = res['sdne'] * np.fabs(res['sdne'])
            self.gps_data.position_covariance[4] = res['sdn']**2
            self.gps_data.position_covariance[5] = res['sdun'] * np.fabs(res['sdun'])
            self.gps_data.position_covariance[6] = res['sdeu'] * np.fabs(res['sdeu'])
            self.gps_data.position_covariance[7] = res['sdun'] * np.fabs(res['sdun'])
            self.gps_data.position_covariance[8] = res['sdu']**2
            self.gps_data.position_covariance_type = NavSatFix.COVARIANCE_TYPE_KNOWN

            rospy.logdebug('lat: %f, lon: %f, alt: %f' % (self.gps_data.latitude, self.gps_data.longitude, self.gps_data.altitude))

    def listener(self):
        """Creates a python node, to publish messages from EMLID Rover as ROS messages"""
        rospy.init_node('power_listener', anonymous=True)

        pub = rospy.Publisher('emlid_gps', NavSatFix, queue_size=100)
        rate = rospy.Rate(50)

        # try to establish connection with the TCP server of emlid
        self.init_sockets()

        # if connection is successful begin collecting data and parsing them
        while not rospy.is_shutdown():

            # try getting data on the channel
            try:

                data = self.socket_listen.recv(1024)
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
                pub.publish(self.gps_data)

            rate.sleep()

if __name__ == '__main__':
    p = EmlidReach()
    try:
        p.listener()
    except rospy.ROSInterruptException:
        p.close_socket()
        rospy.loginfo('Exiting...')
