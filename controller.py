#!/usr/bin/env python3

import rospy
import math
from asr_flir_ptu_driver.msg import State
from geometry_msgs.msg import Point
from std_msgs.msg import Bool

class PTUController:
    def __init__(self):
        self.state_pub = rospy.Publisher('/asr_flir_ptu_driver/state_cmd', State, queue_size=1)
        self.state_sub = rospy.Subscriber('/asr_flir_ptu_driver/ptu_state', State, self.state_callback)
        self.trigger_pub = rospy.Publisher('trigger', Bool, queue_size=10)
        
        self.seq = 0
        self.pan_value = 20.0
        self.tilt_value = -50.0
        self.velocity = 0
        self.effort = 0

        # To keep track of the last position where a trigger was sent
        self.last_triggered_position = None

    def move_ptu(self, pan, tilt, velocity, effort):
        
        # PTU parameter
        self.pan_value = pan
        self.tilt_value = tilt
        self.seq += 1
        self.velocity = velocity
        self.effort = effort

        movement_goal = State()
        movement_goal.state.position = [self.pan_value, self.tilt_value]
        movement_goal.state.velocity = [self.velocity]
        movement_goal.state.effort = [self.effort]
        movement_goal.state.name = ['']
        movement_goal.no_check_forbidden_area = True
        movement_goal.seq_num = self.seq

        rospy.loginfo("Publishing PTU movement goal: pan = %f, tilt = %f, seq = %d", self.pan_value, self.tilt_value, self.seq)
        self.state_pub.publish(movement_goal)

    def state_callback(self, msg):
        rospy.loginfo("Received PTU state update: seq = %d", msg.seq_num)
        if msg.seq_num != self.seq:
            return  # Ignore old or irrelevant messages

        if msg.finished:
            tolerance = 0.1  # Tolerance for checking position accuracy
            max_distance = max(abs(self.pan_value - msg.state.position[0]),
                               abs(self.tilt_value - msg.state.position[1]))

            if max_distance <= tolerance:
                rospy.loginfo("Movement successful. PTU reached the desired position.")
                self.check_and_publish_trigger()
            else:
                rospy.logwarn("Movement failed. PTU did not reach the desired position.")
        else:
            rospy.loginfo("PTU movement in progress...")

    def check_and_publish_trigger(self):
        current_position = (self.pan_value, self.tilt_value)

        # Check if the trigger was already sent for this position
        if self.last_triggered_position is None or self.last_triggered_position != current_position:
            # Update the last triggered position and send the trigger
            self.last_triggered_position = current_position
            self.publish_trigger(True)
        else:
            rospy.loginfo("Trigger already sent for position: pan = %f, tilt = %f", self.pan_value, self.tilt_value)

    def publish_trigger(self, success):
        trigger_msg = Bool()
        trigger_msg.data = success
        self.trigger_pub.publish(trigger_msg)
        rospy.loginfo("Published trigger message: %s", success)
        # Wait for the shot to finish
        rospy.sleep(2)


class XYZPointSubscriber:
    def __init__(self, ptu_controller):
        self.alpha = 0
        self.beta = 0
        self.ptu_controller = ptu_controller
        self.subscriber = rospy.Subscriber('xyz_point', Point, self.callback)

        # PTU Offsets
        self.offset_x = 0       # Offset to center
        self.offset_y = 0    # Offset to camera position
        self.offset_z = 0.225    # Hight to ground

    def callback(self, data):
        x = -(data.x - self.offset_x)
        y = (data.y - self.offset_y)
        h = -(data.z - self.offset_z)

        if y != 0:
            alpha_rad = math.atan(x / y)
            self.alpha = math.degrees(alpha_rad)
        else:
            rospy.logwarn("Received y=0, cannot calculate arctan(x/y)")

        c = math.sqrt(x ** 2 + y ** 2)
        d = math.sqrt(h ** 2 + c ** 2)

        if d != 0:
            beta_rad = math.acos(h / d)
            self.beta = math.degrees(beta_rad)
        else:
            rospy.logwarn("Received distance=0, cannot calculate acos(h/distance)")

        rospy.loginfo("Received point: (%f, %f, %f)", x, y, h)
        rospy.loginfo("Calculated alpha (degrees): %f", self.alpha)
        rospy.loginfo("Calculated beta (degrees): %f", self.beta)

        # Move the PTU to the calculated alpha and beta
        self.ptu_controller.move_ptu(float(self.alpha), float(self.beta - 90), 200.0, 200.0)

if __name__ == '__main__':
    rospy.init_node('ptu_controller', anonymous=True)
    rospy.loginfo("Initializing PTU Controller...")
    
    ptu_controller = PTUController()
    xyz_subscriber = XYZPointSubscriber(ptu_controller)

    rospy.spin()
