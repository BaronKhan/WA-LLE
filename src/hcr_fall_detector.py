#!/usr/bin/env python
import rospy, time, sys, signal, pickle, os, errno, math
from std_msgs.msg import Int8, Float32MultiArray, Float64MultiArray
from random import randrange
from sklearn import svm

# Fall prevention: using threshold method
# Fall detection: checking to see if the limbs are present and straight in the gait analysis

STATE_NORMAL = 0
STATE_FALLING = 1
STATE_FALLEN = 2

FALLEN_ACC_DIFF = 0.3
FALLEN_GYRO_DIFF = 41.5

# Toggle features
using_fall_prevention = True
using_fall_detection = True

falling_state = STATE_NORMAL
pub_falling_state = rospy.Publisher('falling_state', Int8, queue_size=1)

imu_pos = []
gait_raw = []

accel_mag_buf = []
gyro_mag_buf = []
accel_mag_avg = -1
gyro_mag_avg = -1

# svm_clf = None  # The classification model

def kill_process(signal, frame):
  print("Exiting hcr_fall_detector...")
  sys.exit(0)

def change_state(state):
  global falling_state
  falling_state = state
  pub_falling_state.publish(falling_state)
  print("new state: "+str(state))

def update_buffers(accel_mag, gyro_mag):
  # Update the circular buffers for calculating the average of the last 5 samples
  global accel_mag_buf, gyro_mag_buf
  accel_mag_buf.append(accel_mag)
  gyro_mag_buf.append(gyro_mag)
  while len(accel_mag_buf) > 1:
    del accel_mag_buf[0]
  while len(gyro_mag_buf) > 1:
    del gyro_mag_buf[0]

def reset_buffers():
  global accel_mag_buf, gyro_mag_buf, accel_mag_avg, gyro_mag_avg
  accel_mag_avg = -1
  gyro_mag_avg = -1

def leg_is_straight(hip, knee, ankle):
  if hip[0] > 0 and knee[0] > 0:
    # calculate angle from x-axis of hip to knee
    rads = math.atan2(knee[1]-hip[1],knee[0]-hip[0])
    angle = math.degrees(rads)
    return (angle > 45 and angle < 135)
  elif knee[0] > 0 and ankle[0] > 0:
    # calculate angle from x-axis of knee to ankle
    rads = math.atan2(ankle[1]-knee[1],ankle[0]-knee[0])
    angle = math.degrees(rads)
    return (angle > 45 and angle < 135)
  else:
    return False

def on_fallen():
  # Alert emergency communications
  print("alerting emergency comms...")
  os.system("echo 2 | sudo nodejs src/hcr_walker/src/panicButton/index.js")
  print("successfully sent message")
  change_state(STATE_FALLEN)
  time.sleep(5)
  change_state(STATE_NORMAL)

# Limb detection version
def check_fallen():
  global gait_raw
  if len(gait_raw) == 0:
    print("Error: gait_raw is empty. Skipping limb detection...")
    return
  print("Checking if limbs are being detected: gait_raw = "+str(gait_raw))
  # Check if the limbs are being detected in the gait analysis
  limbs_detected = False
  for limb_point in gait_raw:
    if float(limb_point) > 0: # -1 if limb point is not detected
      print("A limb coordinate was detected -> user has probably not fallen")
      limbs_detected = True
      break
  if not limbs_detected:
    print("user has fallen!")
    on_fallen()
    return
  print("Checking if user is standing...")
  # Check if legs are straight
  # Check right leg
  hip, knee, ankle = gait_raw[:3], gait_raw[3:6], gait_raw[6:9]
  right_leg_straight = leg_is_straight(hip, knee, ankle)
  # Check left left
  hip, knee, ankle = gait_raw[9:12], gait_raw[12:15], gait_raw[15:18]
  left_leg_straight = leg_is_straight(hip, knee, ankle)
  if left_leg_straight or right_leg_straight:
    print("detected a straight leg -> user has probably not fallen")
  else:
    print("no straight legs detected -> user has fallen!")
    on_fallen()

def on_falling():
  change_state(STATE_FALLING)
  time.sleep(5)
  check_fallen()
  change_state(STATE_NORMAL)

# Check Falling: Threshold version
def check_falling():
  global accel_mag_avg, gyro_mag_avg, accel_mag_buf, gyro_mag_buf
  ax, ay, az = imu_pos[0], imu_pos[1], imu_pos[2]
  wx, wy, wz = imu_pos[3], imu_pos[4], imu_pos[5]
  accel_mag = math.sqrt((ax*ax) + (ay*ay) + (az*az))
  gyro_mag = math.sqrt((wx*wx) + (wz*wz))
  update_buffers(accel_mag, gyro_mag)
  if accel_mag_avg > 0 and gyro_mag_avg > 0:
    accel_diff = abs(accel_mag - accel_mag_avg)
    gyro_diff = abs(gyro_mag - gyro_mag_avg)
    print("accel_mag="+str(accel_mag)+", \tavg="+str(accel_mag_avg)+"\t|\tgyro_mag="+str(gyro_mag)+",\tavg="+str(gyro_mag_avg))
    # if using_fall_prevention and (accel_diff >= FALLEN_ACC_DIFF or gyro_diff >= FALLEN_GYRO_DIFF):
    if using_fall_prevention and (gyro_diff >= FALLEN_GYRO_DIFF):
      print("Fall is occuring! accel_diff = "+str(accel_diff)+", gyro_diff = "+str(gyro_diff))
      reset_buffers()
      on_falling()
      return
    # Update averages
    accel_mag_avg = sum(accel_mag_buf)/len(accel_mag_buf)
    gyro_mag_avg = sum(gyro_mag_buf)/len(gyro_mag_buf)
  else:
    accel_mag_avg = accel_mag
    gyro_mag_avg = gyro_mag

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

def imu_callback(data):
  global imu_pos
  imu_pos = data.data
  # check_fallen()
  check_falling()

def gait_callback(data):
  global gait_raw
  gait_raw_all = data.data
  # Expecting 54 data points
  assert(len(gait_raw_all) == 54)
  # Only get the leg data (RHip, RKnee, RAnkle, LHip, LKnee, LAnkle)
  gait_raw = gait_raw_all[24:42]

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
  # Check we're in the right directory
  if not (os.path.isdir("src")):
    sys.exit("couldn't find src/ dir (try executing in the top-level workspace dir)")
  print("fall prevention: "+("enabled" if using_fall_prevention else "disabled"))
  print("fall detection: "+("enabled" if using_fall_detection else "disabled"))
  # if using_fall_prevention or using_fall_detection:
  #   load_svm_model()

  while True:
    # if using_fall_prevention or using_fall_detection:
    #   check_falling()
    # Sleep to avoid consuming all the CPU at once
    time.sleep(0.1)
