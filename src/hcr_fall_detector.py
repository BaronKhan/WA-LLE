#!/usr/bin/env python
import rospy, time, sys, signal
from std_msgs.msg import Int8, Float32MultiArray, Float64MultiArray
from random import randrange

STATE_NORMAL = 0
STATE_FALLING = 1
STATE_FALLEN = 2

FALLEN_THRESHOLD = 2.25  #1.5g*1.5g; saves us from having to calc. sqrt

falling_state = STATE_NORMAL
pub_falling_state = rospy.Publisher('falling_state', Int8, queue_size=1)

imu_pos = []
gait_raw = []

def kill_process(signal, frame):
  print("Exiting hcr_fall_detector...")
  sys.exit(0)

def change_state(state):
  global falling_state
  falling_state = STATE_FALLEN
  pub_falling_state.publish(falling_state)
  print("new state: "+str(state))

def on_fallen():
  change_state(STATE_FALLEN)
  time.sleep(5)
  change_state(STATE_NORMAL)

def imu_callback(data):
  global imu_pos
  imu_pos = data.data
  ax, ay, az = imu_pos[0], imu_pos[1], imu_pos[2]
  wx, wy, wz = imu_pos[3], imu_pos[4], imu_pos[5]
  accel_mag = (ax*ax) + (ay*ay) + (az*az)
  if accel_mag >= FALLEN_THRESHOLD:
    print("Fall detected! accel_mag = "+str(accel_mag))
    on_fallen()

def gait_callback():
  global gait_raw
  pass

# Kill process with Ctrl+C
signal.signal(signal.SIGINT, kill_process)

rospy.init_node('hcr_fall_detector', anonymous=True)
rospy.Subscriber("imu_pos", Float32MultiArray, imu_callback)
rospy.Subscriber("gait_raw", Float64MultiArray, gait_callback)

if __name__ == '__main__':
  while True:
    # Begin dummy code
    # if (randrange(10) < 2):
    #   falling_state = (falling_state + 1) % 3
    #   pub_falling_state.publish(falling_state)
    # End dummy code

    # Sleep to avoid consuming all the CPU at once
    time.sleep(0.1)