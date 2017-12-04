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

#include <openpose_ros/OpenPoseHuman.h>
#include <openpose_ros/OpenPoseHumanList.h>
#include <openpose_ros/PointWithProb.h>

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
                                                        openpose_ros::OpenPoseHumanList> ApproxSyncPolicy;

void op_callback(const std::shared_ptr<openpose_ros::OpenPoseHumanList> op_msg)
{
  ROS_INFO("Got a thing!");
}

void callback(const sensor_msgs::ImageConstPtr& image, const std::shared_ptr<const openpose_ros::OpenPoseHumanList> op_msg)
{
  ROS_INFO("Got a synchronized thing!");
  std_msgs::Float64MultiArray array_msg; // Output message
  array_msg.data.resize(54); // 18(COCO model) * 3(dimensional) points

  // -------------GET REGISTERED RGB-D FRAME-------------
  cv_bridge::CvImagePtr cv_ptr;
  try {
    // Convert sensor_msgs:Image to OpenCV image 
    cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8); // TYPE_16UC1);
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  // -------------OPENPOSE TRACKED KEYPOINTS FROM FILE -------------
  if (op_msg->num_humans > 0) // We only care about the first human we see
  {
    for (int i=0; i<18; i++)
    {
      float x = op_msg->human_list[0].body_key_points_with_prob[i].x;
      float y = op_msg->human_list[0].body_key_points_with_prob[i].y;
      float depth = 0; // NEED TO GET ACTUAL PIXEL VALUES
      array_msg.data.push_back(x);
      array_msg.data.push_back(y);
      array_msg.data.push_back(depth);
      // Draw an example circle on the video stream
      if (x < cv_ptr->image.cols && y < cv_ptr->image.rows)
      {
        cv::circle(cv_ptr->image, cv::Point(x, y), 10, cv::Scalar(0,0,0), -1);
        cv::putText(cv_ptr->image, POSE_COCO_BODY_PARTS.at(i), cv::Point(x,y), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255,0,0));
      }
    }
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

  // ros::Subscriber sub = nh.subscribe("/openpose_ros/human_list", 10, op_callback);

  // message_filters::Subscriber<sensor_msgs::Image> image_sub(nh, "/cv_camera/image_raw", q);
  // message_filters::Subscriber<openpose_ros::OpenPoseHumanList> keypoint_sub(nh, "/openpose_ros/human_list", q);
  // message_filters::Synchronizer<ApproxSyncPolicy> sync(ApproxSyncPolicy(q), image_sub, keypoint_sub);
  // sync.registerCallback(boost::bind(&callback, _1, _2));

  // Publish
  gait_pub = nh.advertise<std_msgs::Float64MultiArray>("gait_raw", 1000);
  img_pub = nh.advertise<sensor_msgs::Image>("/hcr_walker/gait/depth_image_annotated", 1000);
  
  ros::spin();

  return 0;
}
