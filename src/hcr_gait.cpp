#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64MultiArray.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "hcr_gait");
  ros::NodeHandle n;
  ros::Rate loop_rate(10);

  ros::Publisher chatter_pub = n.advertise<std_msgs::Float64MultiArray>("gait_raw", 1000); // Topic name, queue size
  
  std_msgs::Float64MultiArray array_msg;
  array_msg.data.resize(10);
  while (ros::ok())
  {
    for (int i = 0; i < 10; i++)
		{
			//assign array a random number between 0 and 255.
			array_msg.data.push_back(rand() % 255);
		}
    ROS_INFO("I published my gait!");

    chatter_pub.publish(array_msg);

    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;
}