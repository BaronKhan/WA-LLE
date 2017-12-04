#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <iostream>

#include "ros/ros.h"

#include "std_msgs/Float64MultiArray.h"

void arrayCallback(const std_msgs::Float64MultiArray::ConstPtr& array);

int main(int argc, char **argv)
{
	ros::init(argc, argv, "hcr_gait_test_listener");
	ros::NodeHandle n;	
	ros::Subscriber sub = n.subscribe("gait_raw_2D", 100, arrayCallback);

	ros::spin();
	return 0;
}

void arrayCallback(const std_msgs::Float64MultiArray::ConstPtr& array)
{
    // print all the remaining numbers
    ROS_INFO("New array:");
	for(std::vector<double>::const_iterator it = array->data.begin(); it != array->data.end(); ++it)
	{
        ROS_INFO("%f", *it);
    }
	return;
}