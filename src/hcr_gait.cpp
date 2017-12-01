#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64MultiArray.h"
#include <sensor_msgs/PointCloud2.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/filters/voxel_grid.h>

ros::Publisher pub;

void cloud_cb (const pcl::PCLPointCloud2ConstPtr& cloud_msg)
{
  // Convert to PCL pointcloud2
  // pcl::PCLPointCloud2 cloud;
  // pcl_conversions::toPCL(*cloud_msg,cloud);

  // pcl::PCLPointCloud2 cloud_filtered;
  // // Perform voxel grid filtering
  // pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  // sor.setInputCloud (cloud.makeShared());
  // sor.setLeafSize (0.1, 0.1, 0.1);
  // sor.filter (cloud_filtered);
 
  // // Convert to pointXYZRGB
  pcl::PCLPointCloud2 pcl_pc2;
  pcl_conversions::toPCL(*cloud_msg, pcl_pc2);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudXYZRGB(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::fromPCLPointCloud2(pcl_pc2,*cloudXYZRGB);

  // std::cout << "X " << cloudXYZRGB->points.x.size() << "Y " << cloudXYZRGB->points.y.size() << "Z " << cloudXYZRGB->points.z
  // // for (int i=0; i<cloudXYZRGB->points.size(); i++)
  // // {
  // //   // if (cloudXYZRGB->points[i].z > )
  // // }
  // pt_cloud->points[i].x;
  // Publish the data
  pub.publish (cloud_msg);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "hcr_gait");
  ros::NodeHandle n;
  ros::Rate loop_rate(10);

  // -------------GET REGISTERED RGB-D FRAME-------------
  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = n.subscribe ("/camera/depth_registered/points", 1, cloud_cb);
  // Create a ROS publisher for the output point cloud
  pub = n.advertise<pcl::PCLPointCloud2> ("output", 1);

  // -------------READ OPENPOSE TRACKING COORDINATES FROM FILE -------------


  // -------------AUGMENT TRACKED POINTS WITH (RBG)D DATA-------------


  // -------------PUBLISH TO GAIT_RAW TOPIC------------- 
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
    // ROS_INFO("I published my gait!");

    chatter_pub.publish(array_msg);

    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;
}