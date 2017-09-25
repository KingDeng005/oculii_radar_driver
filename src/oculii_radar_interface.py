#!/usr/bin/env python

import rospy
import numpy as np
import math
from oculii_radar_driver.msg import *
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from heartbeat_sender import *
from oculii_radar_driver_node import *

class oculii_radar_interface():

    def __init__(self):
        self.frame_id = rospy.get_param('~frame_id','base_link')
        self.trackArray_subcriber = rospy.Subscriber('oculii_radar_trackArray', oculiiRadarTrackArray, self.recv_radar_tracks)
        self.marker_publisher = rospy.Publisher('oculii_track_marker_2', MarkerArray, queue_size=1)
        #self.oculii_node = oculii_radar_driver()

    def recv_radar_tracks(self, track_array):
        marker_array = MarkerArray()

        for track in track_array.tracks:
            marker = Marker()
            marker.header.frame_id = self.frame_id
            marker.header.stamp = rospy.Time.now()
            marker.id = track.Track_ID
            #marker.ns = 'oculii'
            marker.action = Marker.ADD
            marker.lifetime = rospy.Duration(1)
            marker.type = Marker.SPHERE

            marker.pose.position.x = track.Z + 4
            marker.pose.position.y = track.X + 0.2
            marker.pose.position.z = 0

            delta = 0.02
            marker.scale.x = 0.5 + delta
            marker.scale.y = 0.5 + delta
            marker.scale.z = 1

            marker.color.a = 1.0
            marker.color.r = 1.0/255
            marker.color.g = 245.0/255
            marker.color.b = 3.0/255
            marker_array.markers.append(marker)

        self.marker_publisher.publish(marker_array)

if __name__ == "__main__":
    rospy.init_node('oculii_radar_interface')
    _ = oculii_radar_interface()
    rospy.spin()


