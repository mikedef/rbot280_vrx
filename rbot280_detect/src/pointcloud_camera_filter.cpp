// node to filter out camera fov of pointcloud

#include <ros/ros.h>
#include <ros/console.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

using namespace pcl;
using namespace std;

#define __APP_NAME__ "pointcloud_camera_filter.cpp"
typedef PointXYZI PointType;

ros::Publisher _pub_filtered_cloud;
std_msgs::Header _sensor_header;

static bool _using_cloud = false;
static double _camera_theta;
static int _camera_hfov;
static double _leaf_size;

void publishCloud(const ros::Publisher *in_publisher,
			  const PointCloud<PointType>::Ptr in_cloud_to_publish_ptr)
{
  sensor_msgs::PointCloud2 cloud_msg;
  toROSMsg(*in_cloud_to_publish_ptr, cloud_msg);
  cloud_msg.header = _sensor_header;
  in_publisher->publish(cloud_msg);

}

/**                                                                                                                                        
 * @brief filter points around vessel to remove self reflections                                                                           
 * @param [in] in_cloud_ptr     :input point cloud to filter                                                                               
 * @param [in] out_cloud_ptr    :output point cloud to store filtered cloud                                                                
 * @param [in] in_distance      :min distance allowed to keep point                                                                        
 */                                                                                                                             
void filterCloud(const PointCloud<PointType>::Ptr in_cloud_ptr,                                                                 
                 PointCloud<PointType>::Ptr out_cloud_ptr, const double in_distance)                                              
{                                                                                                                            
  out_cloud_ptr->points.clear();                                                                                             
  for (unsigned int i=0; i<in_cloud_ptr->points.size(); i++)                                                                       
    {                                                                                                                                  
      float origin_distance = sqrt(pow(in_cloud_ptr->points[i].x, 2) +                                                                 
                                   pow(in_cloud_ptr->points[i].y, 2));                                                                 
      if (origin_distance > in_distance)                                                                                                
        {                                                                                                                               
          out_cloud_ptr->points.push_back(in_cloud_ptr->points[i]);                                                                    
        }                                                                                                                               
    }                                                                                                                                    
}

/**                                                                                                                                        
 * @brief voxel filter to downsample point cloud                                                                                           
 * @param [in] in_cloud_ptr     :input point cloud to downsample                                                                           
 * @param [in] out_cloud_ptr    :output point cloud to store downsampled cloud                                                             
 * @param [in] in_leaf_size     :leaf size to downsample cloud                                                                             
 */                                                                                                                                        
void downsampledCloud(const PointCloud<PointType>::Ptr in_cloud_ptr,                                                                       
                      PointCloud<PointType>::Ptr out_cloud_ptr, float in_leaf_size = 0.2)                                                  
{                                                                                                                                          
  VoxelGrid<PointType> vg;                                                                                                                 
  vg.setInputCloud(in_cloud_ptr);                                                                                                          
  vg.setLeafSize((float) in_leaf_size, (float) in_leaf_size, (float) in_leaf_size);                                                        
  vg.filter(*out_cloud_ptr);                                                                                                               
}

/**                                                                                                                                        
 * @brief filter out points that are not in the view of the camera. Try using Frustum Culling if it is supported by ROS                    
 * @param [in] in_cloud_ptr     :input pointcloud to filter                                                                                
 * @param [in] out_cloud_ptr    :output pointcloud to store filtered cloud                                                                 
 * @param [in] fov              :horizontal fov of the camera                                                                              
 * @param [in] theta            :pose of the camera w.r.t the camera origin                                                                
   //                   Frustum and the vectors a, b, c and d. T is the position of the camera                                             
   //                        http://docs.ros.org/en/hydro/api/pcl/html/frustum__culling_8hpp_source.html                                   
   //                             _________                                                                                                
   //                           /|       . |                                                                                               
   //                       d  / |   c .   |                                                                                               
   //                         /  | __._____|                                                                                               
   //                        /  /  .      .                                                                                                
   //                 a <---/-/  .    .                                                                                                    
   //                      / / .   .  b                                                                                                    
   //                     /   .                                                                                                            
   //                     .                                                                                                                
   //                   T                                                                                                                  
   //                                                                                                                                      
 */                                                                                                                                        
void filterCameraView(const PointCloud<PointType>::Ptr in_cloud_ptr,                                                                       
                      PointCloud<PointType>::Ptr out_cloud_ptr,                                                                            
                      const int fov,                                                                                                       
                      const double theta)                                                                                                  
{                                                                                                                                          
  float half_fov = float (fov * M_PI / 180) / 2;                                                                                           
  float camera_theta = float (theta * M_PI / 180);                                                                                         
  out_cloud_ptr->points.clear();                                                                                                           
  for (unsigned int i=0; i<in_cloud_ptr->points.size(); i++)                                                                               
    {                                                                                                                                      
      float current_theta = atan2(in_cloud_ptr->points[i].y, in_cloud_ptr->points[i].x);                                                   
      //cout << "current theta: " << current_theta << ", camrera theta: " << camera_theta << ", camera theta-half_fov: " <<                
      //camera_theta-half_fov  << ", camera theta + half_fov: " << camera_theta + half_fov << endl;                                        
      if (current_theta > camera_theta - half_fov && current_theta < camera_theta + half_fov)                                              
        {                                                                                                                                  
          out_cloud_ptr->points.push_back(in_cloud_ptr->points[i]);                                                                        
        }                                                                                                                                  
    }                                                                                                                                      
                                                                                                                                           
}

void pcCallback(const sensor_msgs::PointCloud2ConstPtr& in_cloud)
{
  if (!_using_cloud)
    {
      _using_cloud = true;

      PointCloud<PointType>::Ptr current_cloud_ptr(new PointCloud<PointType>);
      PointCloud<PointType>::Ptr filtered_cloud_ptr(new PointCloud<PointType>);
      PointCloud<PointType>::Ptr downsampled_cloud_ptr(new PointCloud<PointType>);
      PointCloud<PointType>::Ptr camera_cloud_ptr(new PointCloud<PointType>);

      fromROSMsg(*in_cloud, *current_cloud_ptr);
      _sensor_header = in_cloud->header;

      // filter for self reflections
      filterCloud(current_cloud_ptr, filtered_cloud_ptr, 1.0);

      // downsample cloud
      downsampledCloud(filtered_cloud_ptr, downsampled_cloud_ptr, _leaf_size);

      // filter for camera fov
      filterCameraView(downsampled_cloud_ptr, camera_cloud_ptr, _camera_hfov, _camera_theta);

      // publish filtered cloud
      publishCloud(&_pub_filtered_cloud, camera_cloud_ptr);
      
      _using_cloud = false;
    }
}


int main(int argc, char **argv)
{
  // Init ROS
  ros::init (argc, argv, __APP_NAME__);
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  string pc_topic;
  if(private_nh.getParam("pc_topic", pc_topic))
    {
      ROS_INFO("Pointcloud camera filter > Setting pc_topic to %s", pc_topic.c_str());
    }
  else
    {
      ROS_INFO("Pointcloud camera filter > No Pointcloud recieved");
      return 0;
    }

  // Publisher
  _pub_filtered_cloud = nh.advertise<sensor_msgs::PointCloud2>(pc_topic + "/camera_FOV_cloud", 100);

  // Init parameters
  private_nh.param("camera_hfov", _camera_hfov, 80);
  ROS_INFO("[%s] camera_hfov: %d", __APP_NAME__, _camera_hfov);
  private_nh.param("camera_theta", _camera_theta, 0.0);
  ROS_INFO("[%s] camera_theta: %f", __APP_NAME__, _camera_theta);
  private_nh.param("leaf_size", _leaf_size, 0.0);
  ROS_INFO("[%s] leaf_size %f", __APP_NAME__, _leaf_size);

  // Subscribe to the input pointcloud 'pc_topic'
  ros::Subscriber sub = nh.subscribe(pc_topic, 100, pcCallback);

  // Spin
  ros::spin();
}
