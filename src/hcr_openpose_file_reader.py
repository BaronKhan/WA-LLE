#!/usr/bin/env python
import sys # sys.exit
import os # File & directory handling
import json # JSON parsing
import re # Regex
import signal
import rospy
from sensor_msgs.msg import JointState
# import sensor_msgs.point_cloud2 as pc2
import std_msgs

OPENPOSE_OUTPUT_DIR = '/home/junshern/killmenow/'
NUM_MODEL_KEYPOINTS = 15 # Number of keypoints from MPI model
NUM_KEYPOINT_DIM = 3 # Number of dimensions per keypoint
PUBLISH_RATE = 5 # 5Hz

def kill_process(signal, frame):
    print "Exiting hcr_openpose_reader..."
    sys.exit(0)

# def chunks(keypoint_list, chunk_size):
#     """
#     Yield successive chunk_size-sized chunks from keypoint_list.
#     """
#     assert len(keypoint_list) == NUM_MODEL_KEYPOINTS * NUM_KEYPOINT_DIM
#     for i in xrange(0, len(keypoint_list), chunk_size):
#         yield keypoint_list[i:i + chunk_size]

def make_jointstate_msg(keypoints):
    """
    Takes a list of keypoint data and returns it as a JointState Msg
    """
    # Create header
    header = std_msgs.msg.Header()
    header.stamp = rospy.Time.now()
    header.frame_id = 'map'

    kp_msg = JointState()
    kp_msg.header = header
    kp_msg.position = keypoints
    return kp_msg

def main():
    # ROS publisher node setup
    rospy.init_node('hcr_openpose_reader', anonymous=True)
    pub = rospy.Publisher('/hcr_walker/gait/openpose_keypoints', JointState, queue_size=1)
    signal.signal(signal.SIGINT, kill_process) # Kill process with Ctrl+C
    pub_rate = rospy.Rate(PUBLISH_RATE)

    # Initial conditions
    current_file_number = -1
    keypoints = [0] * (NUM_MODEL_KEYPOINTS * NUM_KEYPOINT_DIM)

    # Publishing loop
    while not rospy.is_shutdown():
        try:
            # Get the latest file in the openpose output directory
            output_files = os.listdir(OPENPOSE_OUTPUT_DIR)
            if output_files:
                latest_file_name = sorted(output_files)[-1]
                latest_file_number = int(re.findall(r'\d+', latest_file_name)[0])
                if latest_file_number > current_file_number: # No need to update if no new files
                    current_file_number = latest_file_number
                    # Read JSON file
                    try:
                        file_str = OPENPOSE_OUTPUT_DIR+latest_file_name
                        data = json.load(open(file_str))
                        if data['people']:
                            # Take only the pose (body) keypoints of the first detected person
                            # Output format x1,y1,c1,x2,y2,c2,...
                            keypoints = data['people'][0]['pose_keypoints']
                        # Remove the file after we have read it to reduce memory bloat
                        os.remove(file_str)
                    except OSError:
                        pass
        except OSError:
            pass

        # Always publishing the latest data available
        rospy.loginfo(keypoints)
        pc2_msg = make_jointstate_msg(keypoints)
        rospy.loginfo(pc2_msg)
        pub.publish(pc2_msg)
        pub_rate.sleep()

if __name__ == '__main__':
    main()
