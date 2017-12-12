#!/usr/bin/env python

# Import required Python code.
import roslib
import rospy
import serial
import time

# Import custom message data.
from std_msgs.msg import Int8

brakes = None

# Create a callback function for the subscriber.
def callback(data):
    global brakes
    fallingState = data.data
    print("sending to usb..")
    if fallingState == 0:			#Any other value can be passed as an all clear
        brakes.write ('0') #Apply brakes
        time.sleep(5)
    else:
        brakes.write('1') #Normal activity
        time.sleep(5)
    print("sent to usb")

# This ends up being the main while loop.
def listener():
    # Create a subscriber with appropriate topic, custom message and name of callback function.
    rospy.Subscriber("falling_state", Int8, callback, queue_size=1)
    # Wait for messages on topic, go to callback function when new messages arrive.
    rospy.spin()

# Main function.
if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('arduino_send_node', anonymous = True)
    # Initialize serial connection
    brakes = serial.Serial("/dev/ttyACM0", 9600)	#Setup serial connection
    # Go to the main loop.
    listener()
