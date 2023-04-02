#ifndef LIDAR_DETECTION_TRACK_H_
#define LIDAR_DETECTION_TRACK_H_

#include <iostream>
#include <string>
#include <vector>
#include <mutex>

#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include "autoware_msgs/Centroids.h"
#include "autoware_msgs/CloudCluster.h"
#include "autoware_msgs/CloudClusterArray.h"
#include "autoware_msgs/DetectedObject.h"
#include "autoware_msgs/DetectedObjectArray.h"
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/version.hpp>

#include "../lib/ground_detector/patchwork/patchwork.h"
#include "../lib/visualization/visualize_detected_objects.h"
#include "../lib/pre_process/roi_clip/roi_clip.h"
#include "../lib/euclidean_cluster/euclidean_cluster.h"
#include "../lib/pre_process/voxel_grid_filter/voxel_grid_filter.h"
#include "../lib/bounding_box/bounding_box.h"
#include "../lib/visualization/visualize_detected_objects.h"

#include <geometry_msgs/Twist.h>
#include <tf/transform_broadcaster.h>

#include <Eigen/Eigen>
#include <ctgmath>

class lidarObstacleDetection
{
public:
  lidarObstacleDetection(ros::NodeHandle nh, const ros::NodeHandle& pnh);
  ~lidarObstacleDetection()= default;;

  RoiClip roi_clip_;
  VoxelGridFilter voxel_grid_filter_;
  PatchWork patch_work_;
  EuclideanCluster cluster_;

  BoundingBox bounding_box_;
  VisualizeDetectedObjects vdo_;

  ros::Publisher _pub_clip_cloud;
  ros::Publisher _pub_ground_cloud;
  ros::Publisher _pub_down_cloud;
  ros::Publisher _pub_noground_cloud;
  ros::Publisher _pub_cluster_cloud;
  ros::Publisher _pub_clusters_message;

  ros::Publisher _pub_detected_objects;
  ros::Publisher _pub_detected_3Dobjects;

  ros::Publisher _pub_cluster_visualize_markers;
  ros::Publisher _pub_3Dobjects_visualize_markers;
  ros::Publisher _pub_min_pose;
  ros::Publisher _pub_min_point;

  VisualizeDetectedObjects vdto;

  std::vector<tf2::Vector3> poses_{};


private:
  void ClusterCallback(const sensor_msgs::PointCloud2ConstPtr &in_sensor_cloud);
  void ArmThetaCallback(const sensor_msgs::PointCloud2ConstPtr &in_sensor_cloud);
  void tfBroadcaster();
  void publishDetectedObjects(const autoware_msgs::CloudClusterArray &in_clusters, autoware_msgs::DetectedObjectArray &detected_objects);
};

#endif
