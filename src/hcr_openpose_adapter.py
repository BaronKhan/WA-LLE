#!/usr/bin/env python
import rospy
from sensor_msgs.msg import JointState
from openpose_ros.msg import OpenPoseHuman
from openpose_ros.msg import OpenPoseHumanList
from openpose_ros.msg import PointWithProb
import std_msgs

def make_jointstate_msg(keypoints):
    """
    Takes a list of keypoint data and returns it as a JointState msg
    """
    # Create header
    header = std_msgs.msg.Header()
    header.stamp = rospy.Time.now()
    header.frame_id = 'map'

    kp_msg = JointState()
    kp_msg.header = header
    kp_msg.position = keypoints
    return kp_msg

def callback(data):
    """
    Receive openpose_ros::OpenPoseHumanList data and republish it as sensor_msgs::JointState
    """
    keypoints_list = []
    for point_with_prob in data.human_list[0].body_key_points_with_prob:
        x = point_with_prob.x
        y = point_with_prob.y
        p = point_with_prob.prob
        keypoints_list.extend((x,y,p))
    msg = make_jointstate_msg(keypoints_list)
    pub.publish(msg)

def main():
    rospy.init_node('hcr_openpose_adapter', anonymous=True)
    rospy.Subscriber("/openpose_ros/human_list", OpenPoseHumanList, callback)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

pub = rospy.Publisher('/hcr_walker/gait/openpose_keypoints', JointState, queue_size=1)
if __name__ == '__main__':
    main()