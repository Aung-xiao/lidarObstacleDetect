#ifndef ROI_CLIP_H
#define ROI_CLIP_H

#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <geometry_msgs/PoseArray.h>

class RoiClip
{
public:
  RoiClip(ros::NodeHandle node_handle, ros::NodeHandle private_node_handle);
  ~RoiClip()= default;
  pcl::PointCloud<pcl::PointXYZI>::Ptr GetROI(const pcl::PointCloud<pcl::PointXYZI>::Ptr& in,pcl::PointCloud<pcl::PointXYZI>::Ptr &out,geometry_msgs::PoseArray& pose_array);
  pcl::PointCloud<pcl::PointXYZI>::Ptr ClipVehicle(const pcl::PointCloud<pcl::PointXYZI>::Ptr& in);
  pcl::PointCloud<pcl::PointXYZI>::Ptr ClipArm(const pcl::PointCloud<pcl::PointXYZI>::Ptr& in,geometry_msgs::PoseArray& pose_array);
private:
  double roi_x_min_;
  double roi_x_max_;
  double roi_y_min_;
  double roi_y_max_;
  double roi_z_min_;
  double roi_z_max_;
  double vehicle_x_min_;
  double vehicle_x_max_;
  double vehicle_y_min_;
  double vehicle_y_max_;
  double vehicle_z_min_;
  double vehicle_z_max_;
  static bool IsIn(const float x, const float x_min, const float x_max) {
    return (x < x_max) && (x > x_min);
  }

};

#endif