// %Tag(FULLTEXT)%
// %Tag(ROS_HEADER)%
#include "ros/ros.h"
// %EndTag(ROS_HEADER)%
// %Tag(MSG_HEADER)%
#include "std_msgs/Float32MultiArray.h"
// %EndTag(MSG_HEADER)%

#include <vector>
#include <cstdlib>

int main(int argc, char **argv)
{
// %Tag(INIT)%
  ros::init(argc, argv, "hcr_imu");
// %EndTag(INIT)%

// %Tag(NODEHANDLE)%
  ros::NodeHandle n;
// %EndTag(NODEHANDLE)%

// %Tag(PUBLISHER)%
  ros::Publisher imu_pub = n.advertise<std_msgs::Float32MultiArray>("imu_pos", 1000);
// %EndTag(PUBLISHER)%
  
  int rate = 10;
// %Tag(LOOP_RATE)%
  ros::Rate loop_rate(rate);
// %EndTag(LOOP_RATE)%

// %Tag(ROS_OK)%
  int count = 0;
  while (ros::ok())
  {    
// Initialise array of sample data
  std::vector<float> sample_data;
  sample_data.push_back(1.0);
  sample_data.push_back(2.0);
  sample_data.push_back(3.0);
  sample_data.push_back(4.0);
  sample_data.push_back(5.0);


// %EndTag(ROS_OK)%
// %Tag(FILL_MESSAGE)%
    std_msgs::Float32MultiArray imu_pos_array;
    imu_pos_array.data = sample_data; // Sample data for now
// %EndTag(FILL_MESSAGE)%

// %Tag(ROSCONSOLE)%
    ROS_INFO("%f, ", imu_pos_array.data[0]);
    ROS_INFO("%f, ", imu_pos_array.data[1]);
    ROS_INFO("%f, ", imu_pos_array.data[2]);
    ROS_INFO("%f, ", imu_pos_array.data[3]);
    ROS_INFO("%f", imu_pos_array.data[4]);
// %EndTag(ROSCONSOLE)%

// %Tag(PUBLISH)%
    imu_pub.publish(imu_pos_array);
// %EndTag(PUBLISH)%

// %Tag(SPINONCE)%
    ros::spinOnce();
// %EndTag(SPINONCE)%

// %Tag(RATE_SLEEP)%
    loop_rate.sleep();
// %EndTag(RATE_SLEEP)%
    ++count;
  }


  return 0;
}
// %EndTag(FULLTEXT)%
