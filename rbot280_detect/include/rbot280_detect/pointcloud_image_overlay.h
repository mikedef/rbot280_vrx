

#ifndef PROJECT_POINTCLOUD_IMAGE_OVERLAY_H
#define PROJECT_POINTCLOUD_IMAGE_OVERLAY_H

#define __APP_NAME__ "pointcloud_image_overlay"

#include <string>

#include <ros/ros.h>
#include <ros/console.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <cv_bridge/cv_bridge.h>  // change from ros img msg to CV Mat
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>

class PclImageOverlay
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::SubscriberFilter image_sub_;
  image_transport::Publisher image_pub_;
  ros::Subscriber image_intrinsics_sub_;
  message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_sub_;
  tf::TransformListener* tf_listener_;
  tf::StampedTransform pc_cam_tf_;
  tf::StampedTransform custom_tf_;   // Needed?
  image_geometry::PinholeCameraModel cam_model_;
  


  void InitRosIO(ros::NodeHandle &in_private_nh);
  /**
   * @brief callback to init params and subscribtions
   * @param[in] in_img_msg
   * @param[in] in_cloud_msg
   */

 public:
  void Run();
 PclImageOverlay() : it_(nh_){};
  virtual ~PclImageOverlay();

};
#endif //PROJECT_POINTCLOUD_IMAGE_OVERLAY_H
