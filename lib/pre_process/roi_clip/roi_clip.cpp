#include "roi_clip.h"


RoiClip::RoiClip(ros::NodeHandle node_handle, ros::NodeHandle private_node_handle)
{
  private_node_handle.param("roi_x_min", roi_x_min_, 0.0);
  private_node_handle.param("roi_x_max", roi_x_max_, 100.0);
  private_node_handle.param("roi_y_min", roi_y_min_, -20.0);
  private_node_handle.param("roi_y_max", roi_y_max_, 20.0);
  private_node_handle.param("roi_z_min", roi_z_min_, -1.75);
  private_node_handle.param("roi_z_max", roi_z_max_, 2.0);

  private_node_handle.param("vehicle_x_min", vehicle_x_min_, -1.2);
  private_node_handle.param("vehicle_x_max", vehicle_x_max_, 3.0);
  private_node_handle.param("vehicle_y_min", vehicle_y_min_, -1.0);
  private_node_handle.param("vehicle_y_max", vehicle_y_max_, 1.0);
  private_node_handle.param("vehicle_z_min", vehicle_z_min_, -1.7);
  private_node_handle.param("vehicle_z_max", vehicle_z_max_, 0.2);
}


pcl::PointCloud<pcl::PointXYZI>::Ptr RoiClip::GetROI(const pcl::PointCloud<pcl::PointXYZI>::Ptr& in,pcl::PointCloud<pcl::PointXYZI>::Ptr &out,std::vector<tf2::Vector3> &poses)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr clipVehicle = ClipVehicle(in);
  pcl::PointCloud<pcl::PointXYZI>::Ptr clipArm = ClipArm(clipVehicle,poses);
  if (!clipArm->points.empty())
  {
    for (auto &p : clipArm->points)
    {
      if (IsIn(p.x, roi_x_min_, roi_x_max_) && IsIn(p.y, roi_y_min_, roi_y_max_) && IsIn(p.z, roi_z_min_, roi_z_max_))
        out->push_back(p);
    }
    ROS_INFO("GetROI sucess");
    return out;

  }
  ROS_ERROR("GetROI fail");
  return nullptr;
}


pcl::PointCloud<pcl::PointXYZI>::Ptr RoiClip::ClipVehicle(const pcl::PointCloud<pcl::PointXYZI>::Ptr& in){
  if(!in->points.empty())
  {
    pcl::PointCloud<pcl::PointXYZI>::Ptr out(new pcl::PointCloud<pcl::PointXYZI>);
    for (auto &p : in->points)
    {
      if (IsIn(p.x, vehicle_x_min_, vehicle_x_max_) && IsIn(p.y, vehicle_y_min_, vehicle_y_max_) && IsIn(p.z, vehicle_z_min_, vehicle_z_max_)){

      }
      else out->push_back(p);
    }
    return out;
  }
  return nullptr;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr RoiClip::ClipArm(const pcl::PointCloud<pcl::PointXYZI>::Ptr& in,std::vector<tf2::Vector3> &poses){
  if(!in->points.empty())
  {
    pcl::PointCloud<pcl::PointXYZI>::Ptr out(new pcl::PointCloud<pcl::PointXYZI>);
    for (auto &p : in->points)
    {
      if(poses.empty())
      {
        ROS_INFO("Get arm pose fail!\r\n");
        out->push_back(p);
      }
      else{
        for(auto &pose: poses)
        {
          if (IsIn(p.x, pose.x(), pose.x()+0.1) && IsIn(p.y, pose.y(), pose.y()+0.1) && IsIn(p.z, pose.z(), pose.z()+0.1)){
          }
          else out->push_back(p);
        }
      }
    }
    return out;
  }
  return nullptr;
}
