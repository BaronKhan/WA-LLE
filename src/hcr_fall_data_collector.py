#!/usr/bin/env python
import rospy, time, sys, signal, csv, os, errno
from std_msgs.msg import Float32MultiArray, Float64MultiArray

# Get keyboard input
# https://ubuntuforums.org/showthread.php?t=1514035&p=9488711#post9488711
import curses

STATE_NORMAL = 0
STATE_FALLING = 1
STATE_FALLEN = 2

imu_pos = []
gait_raw = []
falling_state = STATE_NORMAL  # To be observed and recorded

def kill_process(signal, frame):
  print("Exiting hcr_fall_data_collector...")
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

def add_data(csv_file, win):
  global imu_pos, gait_raw, falling_state
  if (len(imu_pos) > 0) and (len(gait_raw) > 0):
    row = list(imu_pos + gait_raw)
    row.append(falling_state)
    print("\rWriting row: "+str(row))
    writer = csv.writer(csv_file)
    writer.writerow(row)

def poll_falling_state(win):
  global falling_state, gait_raw
  try:
    key = win.getkey()
  except: # In no delay mode getkey raises an exception if no key is pressed
    key = None
    falling_state = STATE_NORMAL
  if key == "f":
    falling_state = STATE_FALLING
  elif key == "d":
    falling_state = STATE_FALLEN
    gait_raw = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]  # redundant data


rospy.init_node('hcr_fall_data_collector', anonymous=True)
rospy.Subscriber("imu_pos", Float32MultiArray, imu_callback)
rospy.Subscriber("gait_raw", Float64MultiArray, gait_callback)

def main(win):
  win.nodelay(True) # Make getkey() not wait
  csv_file = init_data()
  while True:
    poll_falling_state(win)
    curses.flushinp() # clear input buffer (it built up while sleeping)
    add_data(csv_file, win)
    # Sleep to avoid consuming all the CPU at once
    time.sleep(0.1)

if __name__ == '__main__':
  curses.wrapper(main)