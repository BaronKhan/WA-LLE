#!/usr/bin/env python
import rospy, time, sys, signal
from std_msgs.msg import Bool, String

def kill_process(signal, frame):
  print("\nExiting fall_detection...")
  sys.exit(0)

# Kill process with Ctrl+C
signal.signal(signal.SIGINT, kill_process)

rospy.init_node('fall_detection', anonymous=True)
pub_has_fallen = rospy.Publisher('has_fallen', Bool, queue_size=10)
has_fallen = False

pub_is_falling = rospy.Publisher('is_falling', Bool, queue_size=10)
is_falling = False

def HasFallen():
  pass

def AboutToFall():
  pass

if __name__ == '__main__':
  while True:
    # Sleep to avoid consuming all the CPU at once
    time.sleep(0.2)