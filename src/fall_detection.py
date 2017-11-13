#!/usr/bin/env python
import rospy, time, sys, signal
from std_msgs.msg import Bool, String

def signal_handler(signal, frame):
  print("\nExiting fall_detection...")
  sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

if __name__ == '__main__':
  rospy.init_node('fall_detection', anonymous=True)
  fall_detected_pub = rospy.Publisher('fall_detected', Bool, queue_size=10)
  fall_detected = False
  while True:
    fall_detected = not fall_detected
    fall_detected_pub.publish(fall_detected)
    # Sleep to avoid consuming all the CPU at once
    time.sleep(0.2)