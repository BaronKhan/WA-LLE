#!/usr/bin/env python
import rospy, time, sys, signal
from std_msgs.msg import Int8, Float32MultiArray, Float64MultiArray
from random import randrange

STATE_NORMAL = 0
STATE_FALLING = 1
STATE_FALLEN = 2

def kill_process(signal, frame):
  print("Exiting hcr_fall_detector...")
  sys.exit(0)

def imu_callback():
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

falling_state = STATE_NORMAL
pub_falling_state = rospy.Publisher('falling_state', Int8, queue_size=10)

if __name__ == '__main__':
  while True:
    # Begin dummy code
    if (randrange(10) < 2):
      falling_state = (falling_state + 1) % 3
      pub_falling_state.publish(falling_state)
    # End dummy code

    # Sleep to avoid consuming all the CPU at once
    time.sleep(0.1)