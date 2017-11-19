#!/usr/bin/env python
import rospy, time, sys, signal
from std_msgs.msg import Float32MultiArray, Float64MultiArray

def kill_process(signal, frame):
  print("Exiting hcr_fall_detector...")
  sys.exit(0)


def imu_callback(data):
  pass

def gait_callback():
  pass

# Kill process with Ctrl+C
signal.signal(signal.SIGINT, kill_process)

rospy.init_node('hcr_fall_detector', anonymous=True)

imu_pos = []
rospy.Subscriber("imu_pos", Float32MultiArray, imu_callback)

gait_raw = []
rospy.Subscriber("gait_raw", Float64MultiArray, gait_callback)

if __name__ == '__main__':
  while True:

    # Sleep to avoid consuming all the CPU at once
    time.sleep(0.1)