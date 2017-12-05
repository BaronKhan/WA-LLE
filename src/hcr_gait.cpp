#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <ros/ros.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/JointState.h>
#include "std_msgs/Float64MultiArray.h"

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

// #define NUM_BODY_PARTS 15 // For MPI
#define NUM_BODY_PARTS 18 // For COCO

static const std::string OPENCV_WINDOW = "Image window";
const std::map<unsigned int, std::string> POSE_MPI_BODY_PARTS {
        {0,  "Head"},
        {1,  "Neck"},
        {2,  "RShoulder"},
        {3,  "RElbow"},
        {4,  "RWrist"},
        {5,  "LShoulder"},
        {6,  "LElbow"},
        {7,  "LWrist"},
        {8,  "RHip"},
        {9,  "RKnee"},
        {10, "RAnkle"},
        {11, "LHip"},
        {12, "LKnee"},
        {13, "LAnkle"},
        {14, "Chest"},
        {15, "Background"}
};
const std::map<unsigned int, std::string> POSE_COCO_BODY_PARTS {
        {0,  "Nose"},
        {1,  "Neck"},
        {2,  "RShoulder"},
        {3,  "RElbow"},
        {4,  "RWrist"},
        {5,  "LShoulder"},
        {6,  "LElbow"},
        {7,  "LWrist"},
        {8,  "RHip"},
        {9,  "RKnee"},
        {10, "RAnkle"},
        {11, "LHip"},
        {12, "LKnee"},
        {13, "LAnkle"},
        {14, "REye"},
        {15, "LEye"},
        {16, "REar"},
        {17, "LEar"},
        {18, "Background"}
};
ros::Publisher gait_pub;
ros::Publisher img_pub;
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, 
                                                        sensor_msgs::JointState> ApproxSyncPolicy;


void callback(const sensor_msgs::ImageConstPtr& image, const sensor_msgs::JointStateConstPtr& keypoints)
{
  ROS_INFO("Got a synchronized thing!");
  std_msgs::Float64MultiArray array_msg; // Output message
  array_msg.data.resize(NUM_BODY_PARTS*3); // Num joints to track * 3(dimensional) points

  // -------------GET REGISTERED RGB-D FRAME-------------
  cv_bridge::CvImagePtr cv_ptr;
  try {
    // Convert sensor_msgs:Image to OpenCV image 
    cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::TYPE_8UC1); //BGR8);
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  // -------------OPENPOSE TRACKED KEYPOINTS FROM FILE -------------
  for (int i=0; i<NUM_BODY_PARTS; i++)
  {
    float x = keypoints->position[i*3];
    float y = keypoints->position[i*3+1];
    float depth;
    if (x>=0 && y>=0) depth = cv_ptr->image.at<char>(cv::Point(x, y));
    else depth = -1;
    array_msg.data[i*3] = x;
    array_msg.data[i*3+1] = y;
    array_msg.data[i*3+2] = depth;
    // Draw an example circle on the video stream
    if (x < cv_ptr->image.cols && y < cv_ptr->image.rows)
    {
      cv::circle(cv_ptr->image, cv::Point(x, y), 2, cv::Scalar(255), -1);
      cv::putText(cv_ptr->image, std::to_string((int)depth)/*POSE_COCO_BODY_PARTS.at(i)*/, cv::Point(x,y), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255,0,0));
    }
    ROS_INFO("Body part number %d has x=%f, y=%f, d=%f", i, x, y, depth);
  }
  
  // -------------PUBLISH OUTPUT DATA------------- //
  gait_pub.publish(array_msg);
  img_pub.publish(cv_ptr->toImageMsg());
  // ros::spinOnce();
  // Update GUI Window
  cv::imshow(OPENCV_WINDOW, cv_ptr->image);
  cv::waitKey(3);

  // loop_rate.sleep();
	return;
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "hcr_gait");

  int q = 5; // queue size
  ros::NodeHandle nh;

  // cv::namedWindow(OPENCV_WINDOW);

  message_filters::Subscriber<sensor_msgs::Image> image_sub(nh, "/camera/depth/image_rect_raw", q); //"/cv_camera/image_raw", q);
  message_filters::Subscriber<sensor_msgs::JointState> keypoint_sub(nh, "/hcr_walker/gait/openpose_keypoints", q);
  message_filters::Synchronizer<ApproxSyncPolicy> sync(ApproxSyncPolicy(q), image_sub, keypoint_sub);
  sync.registerCallback(boost::bind(&callback, _1, _2));

  // Publish
  gait_pub = nh.advertise<std_msgs::Float64MultiArray>("gait_raw", 1000);
  img_pub = nh.advertise<sensor_msgs::Image>("/hcr_walker/gait/depth_image_annotated", 1000);
  
  ros::spin();

  return 0;
}
