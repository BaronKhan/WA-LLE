#!/usr/bin/env python
   
import rospy
from std_msgs.msg import Float32MultiArray
 
def hcr_imu():
    pub = rospy.Publisher('imu_pos', Float32MultiArray, queue_size=1000)
    rospy.init_node('hcr_imu', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        hello_str = Float32MultiArray()
	hello_str.data = [1.0, 2.0, 3.0, 4.0, 5.0]
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()

if __name__ == '__main__':
    try:
        hcr_imu()
    except rospy.ROSInterruptException:
        pass
