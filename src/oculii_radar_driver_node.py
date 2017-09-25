#!/usr/bin/env python
# created by Fuheng Deng on 7/11/2017

from oculii_radar_driver.msg import *
import socket
import rospy
import time
import sys
import numpy as np
import struct

TCP_IP = '192.168.2.2'
UDP_IP = '255.255.255.255'
TCP_PORT = 51716
UDP_PORT = 9931
BUFFER_SIZE = 1024
start_command = bytearray('b\x01\x61\x62\xFF\x9E')
stop_command = bytearray('b\x01\x62\x63\xEF\x9D')
load_angle = bytearray('b\x01\x32\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00')

# define match table
FRAME_SIZE_TABLE = ['', '320x240', '640x480', '1280x720', '1920x1080']
SPEED_TRIGGER_FLAG_TABLE = ['above','above & not in range', 'above & in range', 'below & in range']

class oculii_radar_driver():

    def __init__(self):
        self.TCP_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)  # TCP socket
        self.UDP_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)   # UDP socket
        self.status_publisher = rospy.Publisher('oculii_radar_status',oculiiRadarStatus, queue_size = 10)
        self.track_publisher = rospy.Publisher('oculii_radar_track',oculiiRadarTrack, queue_size = 10)
        self.trackArray_publisher = rospy.Publisher('oculii_radar_trackArray',oculiiRadarTrackArray, queue_size = 10)
        self.connect()
        self.start_data_stream()
        self.get_and_publish_data()

    def connect(self):
        self.TCP_socket.connect((TCP_IP, TCP_PORT))
        self.UDP_socket.bind((UDP_IP, UDP_PORT))

    def start_data_stream(self):
        self.TCP_socket.send(start_command)

    def stop_data_stream(self):
        self.TCP_socket.send(stop_command)

    def get_and_publish_data(self):
        while True:
            raw_data = self.UDP_socket.recv(BUFFER_SIZE)
            data = self.get_formatted_data(raw_data)
            num_obj = 0
            #print(data)
            if self.is_status_packet(data):
                status_msg = oculiiRadarStatus()
                status_msg.header.stamp = rospy.Time.now() 
                status_msg.Radar_Alive = str(data[2] == 0x01)
                status_msg.Frame_Size = FRAME_SIZE_TABLE[data[3]]
                status_msg.Frame_Rate = data[4]
                status_msg.GPS = str(data[5] == 0x01)
                status_msg.nTracks = num_obj = data[7]
                self.status_publisher.publish(status_msg)
            track_array = oculiiRadarTrackArray()
            for cnt in range(num_obj):
                raw_data = self.UDP_socket.recv(BUFFER_SIZE)
                data = self.get_formatted_data(raw_data)
                if self.is_track_packet(data):
                    track_msg = oculiiRadarTrack()
                    track_msg.header.stamp = rospy.Time.now() 
                    track_msg.Sensor_ID = self.unpack_bytes(data[2:4], 's')
                    track_msg.Track_ID = self.unpack_bytes(data[4:8], 'i')
                    track_msg.X = self.unpack_bytes(data[8:12], 'f')
                    track_msg.Y = self.unpack_bytes(data[12:16], 'f')
                    track_msg.Z = self.unpack_bytes(data[16:20], 'f')
                    track_msg.Speed = self.unpack_bytes(data[20:24], 'f')
                    track_msg.Horizontal_RCS = self.unpack_bytes(data[24:26], 's')
                    track_msg.Vertical_RCS = self.unpack_bytes(data[26:28], 's')
                    track_msg.Speed_Trigger_Flag = SPEED_TRIGGER_FLAG_TABLE[data[28]]
                    track_msg.Lane = data[29]
                    track_msg.Track_Packet_Counter = data[30]
                    track_array.tracks.append(track_msg) # add the track msg to the array
                    self.track_publisher.publish(track_msg)
            if len(track_array.tracks) != 0:
                track_array.header.stamp = rospy.Time.now() 
                self.trackArray_publisher.publish(track_array)

    # helper function from here
    @staticmethod
    def is_status_packet(data):
        return data[0:2] == [0x01, 0x11]

    @staticmethod
    def is_track_packet(data):
        return data[0:2] == [0x01, 0x21]

    @staticmethod
    def get_formatted_data(data):
        res = []
        for c in data:
            res.append(int(ord(c)))
        return res

    @staticmethod
    def unpack_bytes(bytes, type):
        byte_array = bytearray(bytes)
        if type == 's':
            return struct.unpack('<H', byte_array)[0]
        if type == 'i':
            return struct.unpack('<I', byte_array)[0]
        if type == 'f':
            return struct.unpack('<f', byte_array)[0]

    @staticmethod
    def reverse_bytes(bytes):
        return bytes[::-1]

if __name__ == "__main__":
    rospy.init_node('oculii_radar_driver', log_level=rospy.DEBUG)
    ocu = oculii_radar_driver()
    #ocu.get_and_publish_data()
    #rospy.spin()
    #rospy.on_shutdown(ocu.stop_data_stream)  # stop data stream upon closing
