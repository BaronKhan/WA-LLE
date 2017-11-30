#!/usr/bin/env python
  
import rospy, sensortag, signal, sys, time
from std_msgs.msg import Float32MultiArray

# Unsure if the below is needed
# tag = sensortag.SensorTag('24:71:89:E6:30:81')    
# tag.disconnect()
# del tag

def hcr_imu():
    mac_address = '24:71:89:E6:30:81'
    pub = rospy.Publisher('imu_pos', Float32MultiArray, queue_size=1000)
    rospy.init_node('hcr_imu', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    tag = sensortag.SensorTag(mac_address)
    time.sleep(1.0)
    tag.accelerometer.enable()
    while not rospy.is_shutdown():
        imuData = Float32MultiArray()
	imuData.data = tag.accelerometer.read()
        rospy.loginfo(imuData)
        pub.publish(imuData)
        rate.sleep()

def kill_process(signal, frame):    
    print("Exiting hcr_imu...")
    sys.exit(0)

# Kill process with Ctrl+C (Thanks Baron!)
signal.signal(signal.SIGINT, kill_process)

if __name__ == '__main__':
    try:
        hcr_imu()
    except rospy.ROSInterruptException:
        pass
