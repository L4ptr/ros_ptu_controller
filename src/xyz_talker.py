#!/usr/bin/env python3

import rospy
import json
import time
from geometry_msgs.msg import Point
from std_msgs.msg import Bool

class CoordinateSender:
    def __init__(self, points_file):
        self.points_file = points_file
        self.load_points()
        self.point_index = 0

        self.point_pub = rospy.Publisher('xyz_point', Point, queue_size=10)
        self.trigger_sub = rospy.Subscriber('trigger', Bool, self.trigger_callback)
        
        rospy.loginfo("Coordinate Sender initialized. Waiting for trigger...")

    def load_points(self):
        with open(self.points_file, 'r') as f:
            self.points = json.load(f)
        rospy.loginfo("Loaded points: %s", self.points)

    def publish_next_point(self):
        if self.point_index < len(self.points):
            point = self.points[self.point_index]
            point_msg = Point(x=point['x'], y=point['y'], z=point['z'])
            self.point_pub.publish(point_msg)
            rospy.loginfo("Published point %d: %s", self.point_index + 1, point)
            self.point_index += 1
        else:
            rospy.loginfo("All points have been published.")
            rospy.signal_shutdown("Finished publishing all points.")

    def trigger_callback(self, msg):
        if msg.data:
            rospy.loginfo("Received trigger signal. Publishing next point...")
            time.sleep(1)
            self.publish_next_point()

if __name__ == '__main__':
    rospy.init_node('coordinate_sender', anonymous=True)
    points_file = rospy.get_param('~points_file', '/home/bseeger/catkin_ws/src/ros_ptu_controller/src/misc/fre_points.json')
    coordinate_sender = CoordinateSender(points_file)
    rospy.spin()
