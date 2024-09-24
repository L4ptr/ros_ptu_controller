#!/usr/bin/env python3

import rospy
import json
from geometry_msgs.msg import Point

class CoordinateSender:
    def __init__(self, points_file, publish_rate):
        self.points_file = points_file
        self.load_points()
        self.point_index = 0
        self.publish_rate = publish_rate

        self.point_pub = rospy.Publisher('xyz_point', Point, queue_size=10)
        
        rospy.loginfo("Coordinate Sender initialized. Starting to publish points...")

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
            self.point_index = 0

    def start_publishing(self):
        rate = rospy.Rate(self.publish_rate)  # Publish rate in Hz
        while not rospy.is_shutdown():
            self.publish_next_point()
            rate.sleep()

if __name__ == '__main__':
    rospy.init_node('coordinate_sender', anonymous=True)
    points_file = rospy.get_param('~points_file', '/home/bseeger/catkin_ws/src/ros_ptu_controller/src/misc/circular_points.json')
    publish_rate = rospy.get_param('~publish_rate', 2)  # Default to 0.3 Hz if not specified
    coordinate_sender = CoordinateSender(points_file, publish_rate)
    coordinate_sender.start_publishing()
