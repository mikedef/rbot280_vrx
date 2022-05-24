


#include "rbot280_detect/pointcloud_image_overlay.h"



void PclImageOverlay::InitRosIO(ros::NodeHandle &in_private_nh)
{
  std::string img_src, img_info_src, cloud_src;
  in_private_nh.getParam("img_src", img_src);
  ROS_INFO("[%s] img_src: %s", __APP_NAME__, img_src.c_str());
  in_private_nh.getParam("img_info_src", img_info_src);
  ROS_INFO("[%s] img_info_src %s", __APP_NAME__, img_info_src.c_str());
  in_private_nh.getParam("cloud_src", cloud_src);
  ROS_INFO("[%s] cloud_src %s", __APP_NAME__, cloud_src.c_str());
}

void PclImageOverlay::Run()
{
  ros::NodeHandle private_nh("~");
  tf::TransformListener tf_listener;
  tf_listener_ = &tf_listener;

  InitRosIO(private_nh);

  ros::spin()
}

PclImageOverlay::~PclImageOverlay()
{
  // Destructor
}
