#!/usr/bin/env python3

import rospy
import serial
from std_msgs.msg import Bool

# Serial port settings
arduino_port = '/dev/ttyACM0'
baud_rate = 9600

def send_trigger_signal():
    with serial.Serial(arduino_port, baud_rate) as ser:
        ser.write(b'1') # Send the trigger signal

def trigger_callback(msg):
    if msg.data:
        send_trigger_signal()

if __name__ == '__main__':
    rospy.init_node('trigger_listener', anonymous=True)
    rospy.Subscriber('trigger', Bool, trigger_callback)
    rospy.spin()