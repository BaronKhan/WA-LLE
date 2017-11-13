// %Tag(FULLTEXT)%
#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"

#include <cstdlib>
#include <vector>
// %Tag(CALLBACK)%
void chatterCallback(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
  for (int i=0;i<5;i++){
  	ROS_INFO("I heard: [%f]", msg->data[i]);
  }
}
// %EndTag(CALLBACK)%

int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener");
  ros::NodeHandle n;

// %Tag(SUBSCRIBER)%
  ros::Subscriber sub = n.subscribe("imu_pos", 1000, chatterCallback);
// %EndTag(SUBSCRIBER)%

// %Tag(SPIN)%
  ros::spin();
// %EndTag(SPIN)%

  return 0;
}
// %EndTag(FULLTEXT)%
