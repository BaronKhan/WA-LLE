#!/usr/bin/env python
import rospy, time, sys, signal, csv, os, errno
from std_msgs.msg import Float32MultiArray, Float64MultiArray

STATE_NORMAL = 0
STATE_FALLING = 1

imu_pos = []
gait_raw = []
falling_state = STATE_NORMAL  # To be observed and recorded

def kill_process(signal, frame):
  print("Exiting hcr_fall_detector...")
  sys.exit(0)

# Kill process with Ctrl+C
signal.signal(signal.SIGINT, kill_process)

def imu_callback(data):
  global imu_pos
  imu_pos = data.data

def gait_callback(data):
  global gait_raw
  gait_raw = data.data

def init_data():
  # Returns ref to csv writer object
  # Create directory w/ if it doesn't exist
  # Apparently there's a race condition with this -.-
  # https://stackoverflow.com/a/5032238
  try:
    os.makedirs("w")
  except OSError as exception:
    if exception.errno != errno.EEXIST:
      raise
  # We want to create a new file, not overwrite existing ones
  # Check for files fall_data_0.csv, fall_data_1.csv, etc.
  file_num = 0
  while (os.path.isfile("w/fall_data_"+str(file_num)+".csv")):
    file_num += 1
  file_name = "w/fall_data_"+str(file_num)+".csv"
  print("Writing data to "+str(file_name))
  return open(file_name, "wb")

def add_data(csv_file):
  global imu_pos, gait_raw, falling_state
  if (len(imu_pos) > 0) and (len(gait_raw) > 0):
    row = list(imu_pos + gait_raw)
    row.append(falling_state)
    print("Writing row: "+str(row))
    writer = csv.writer(csv_file)
    writer.writerow(row)

rospy.init_node('hcr_fall_data_collection', anonymous=True)
rospy.Subscriber("imu_pos", Float32MultiArray, imu_callback)
rospy.Subscriber("gait_raw", Float64MultiArray, gait_callback)

if __name__ == '__main__':
  csv_file = init_data()
  while True:
    add_data(csv_file)
    # Sleep to avoid consuming all the CPU at once
    time.sleep(0.1)