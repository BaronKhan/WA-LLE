#!/usr/bin/env python
import rospy, time, sys, signal, pickle, os, errno, math
from std_msgs.msg import Int8, Float32MultiArray, Float64MultiArray
from random import randrange
from sklearn import svm

# Fall prevention: using threshold method
# Fall detection: detect whether limbs are present in gait analysis after fall prevention fails

STATE_NORMAL = 0
STATE_FALLING = 1
STATE_FALLEN = 2

FALLEN_ACC_DIFF = 0.3
FALLEN_GYRO_DIFF = 50.0

# Toggle features
using_fall_prevention = True
using_fall_detection = False

falling_state = STATE_NORMAL
pub_falling_state = rospy.Publisher('falling_state', Int8, queue_size=1)

imu_pos = []
gait_raw = []

accel_mag_avg = -1
gyro_mag_avg = -1

# svm_clf = None  # The classification model

def kill_process(signal, frame):
  print("Exiting hcr_fall_detector...")
  sys.exit(0)

def change_state(state):
  global falling_state
  falling_state = STATE_FALLEN
  pub_falling_state.publish(falling_state)
  print("new state: "+str(state))

def on_falling():
  change_state(STATE_FALLING)
  time.sleep(5)
  change_state(STATE_NORMAL)

# Check Falling: Threshold version
def check_falling():
  global accel_mag_old, gyro_mag_old
  ax, ay, az = imu_pos[0], imu_pos[1], imu_pos[2]
  wx, wy, wz = imu_pos[3], imu_pos[4], imu_pos[5]
  accel_mag = math.sqrt((ax*ax) + (ay*ay) + (az*az))
  gyro_mag = math.sqrt((wx*wx) + (wz*wz))
  if accel_mag_old > 0 and gyro_mag_old > 0:
    accel_diff = abs(accel_mag-accel_mag_old)
    gyro_diff = abs(gyro_mag-gyro_mag_old)
    print("accel_diff = "+str(accel_diff)+", gyro_diff = "+str(gyro_diff))
    # if using_fall_prevention and (accel_diff >= FALLEN_ACC_DIFF or gyro_diff >= FALLEN_GYRO_DIFF):
    if using_fall_prevention and (gyro_diff >= FALLEN_GYRO_DIFF):
      print("Fall is occuring! accel_mag = "+str(accel_mag)+", gyro_mag = "+str(gyro_mag))
      on_falling()
  accel_mag_old = accel_mag
  gyro_mag_old = gyro_mag

# Check Falling: SVM version
# def check_falling():
#   global imu_pos, gait_raw, svm_clf
#   if len(imu_pos) == 0 or len(gait_raw) == 0:
#     return
#   if svm_clf:
#     fall_data=[imu_pos+gait_raw]
#     y=round(svm_clf.predict(fall_data))
#     if y == 1 and using_fall_prevention:
#       print("User is falling!")
#       on_falling()
#     gait_raw = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0] # Replace gait data with zeros
#     fall_data=[imu_pos+gait_raw]
#     y=round(svm_clf.predict(fall_data))
#     if y == 2 and using_fall_detection:
#       print("User has fallen!")
#       on_fallen()

def on_fallen():
  change_state(STATE_FALLEN)
  time.sleep(5)
  change_state(STATE_NORMAL)

def check_fallen():
  pass

def imu_callback(data):
  global imu_pos
  imu_pos = data.data
  # check_fallen()
  check_falling()

def gait_callback(data):
  global gait_raw
  gait_raw = data.data

# def load_svm_model():
#   global svm_clf, using_fall_prevention
#   # model path should be the first argument of the python program (w/svm_0.mdl)
#   file_count = 0
#   if (len(sys.argv) > 1):
#     if (os.path.isfile(sys.argv[1])):
#       try:
#         svm_clf = pickle.load(open(sys.argv[1], "rb"))
#         print("loaded "+sys.argv[1])
#       except pickle.UnpicklingError:
#         raise
#       except Exception as e:
#         raise pickle.UnpicklingError(repr(e))
#     else:
#       using_fall_prevention = False
#       print(sys.argv[1]+" is not a valid path to a model\nfall prevention: disabled")
#   else:
#     print("warning: no svm model specified\nsvm_0.mdl will be loaded")
#     if (os.path.isfile("w/svm_0.mdl")):
#       try:
#         svm_clf = pickle.load(open("w/svm_0.mdl", "rb"))
#         print("loaded svm_0.mdl")
#       except pickle.UnpicklingError:
#         raise
#       except Exception as e:
#         raise pickle.UnpicklingError(repr(e))
#     else:
#       using_fall_prevention = False
#       print("could not load svm_0.mdl\nfall prevention: disabled")

# Kill process with Ctrl+C
signal.signal(signal.SIGINT, kill_process)

rospy.init_node('hcr_fall_detector', anonymous=True)
rospy.Subscriber("imu_pos", Float32MultiArray, imu_callback, queue_size=1)
rospy.Subscriber("gait_raw", Float64MultiArray, gait_callback, queue_size=1)

if __name__ == '__main__':
  print("fall prevention: "+("enabled" if using_fall_prevention else "disabled"))
  print("fall detection: "+("enabled" if using_fall_detection else "disabled"))
  # if using_fall_prevention or using_fall_detection:
  #   load_svm_model()
  while True:
    # if using_fall_prevention or using_fall_detection:
    #   check_falling()
    # Sleep to avoid consuming all the CPU at once
    time.sleep(0.1)