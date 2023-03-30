/*
 * @Author: xiaohu
 * @Date: 2022-04-02 00:26:55
 * @Last Modified by: xiaohu
 * @Last Modified time: 2022-04-02 01:12:59
 */

#include "lidar_obstacle_detection.h"

#include <utility>

// 时间统计
int64_t euclidean_time = 0.;
int64_t total_time = 0.;
int counter = 0;

int64_t gtm()
{
  struct timeval tm;
  gettimeofday(&tm, 0);
  int64_t re = (((int64_t)tm.tv_sec) * 1000 * 1000 + tm.tv_usec);
  return re;
}

void publishCloud(
    const ros::Publisher *in_publisher, std_msgs::Header header,
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& in_cloud_to_publish_ptr)
{
  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(*in_cloud_to_publish_ptr, cloud_msg);
  cloud_msg.header = std::move(header);
  in_publisher->publish(cloud_msg);
}

lidarObstacleDetection::lidarObstacleDetection(ros::NodeHandle nh, const ros::NodeHandle& pnh)
    : roi_clip_(nh, pnh), voxel_grid_filter_(nh, pnh), cluster_(nh, pnh), bounding_box_(pnh)
{
  ros::Subscriber sub = nh.subscribe("/livox_horizon_points", 1, &lidarObstacleDetection::ClusterCallback, this);
  ///livox_horizon_points
  ///livox/lidar

  _pub_clip_cloud = nh.advertise<sensor_msgs::PointCloud2>("/points_clip", 1);
  _pub_down_cloud = nh.advertise<sensor_msgs::PointCloud2>("/points_down", 1);
  _pub_noground_cloud= nh.advertise<sensor_msgs::PointCloud2>("/points_noground", 1);
  _pub_cluster_cloud = nh.advertise<sensor_msgs::PointCloud2>("/points_cluster", 1);
  _pub_clusters_message = nh.advertise<autoware_msgs::CloudClusterArray>("/detection/lidar_detector/cloud_clusters", 1);
  _pub_detected_objects = nh.advertise<autoware_msgs::DetectedObjectArray>("/detection/lidar_detector/objects", 1);
  _pub_cluster_visualize_markers = nh.advertise<visualization_msgs::MarkerArray>("/visualize/cluster_markers", 1);
  _pub_min_pose=nh.advertise<geometry_msgs::Pose>("/min_pose",1);
  ros::spin();
}

void lidarObstacleDetection::ClusterCallback(
    const sensor_msgs::PointCloud2ConstPtr &in_sensor_cloud)
{
  std_msgs::Header header = in_sensor_cloud->header;
  pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr clip_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr downsampled_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr noground_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr ground_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);

  pcl::fromROSMsg(*in_sensor_cloud, *in_cloud_ptr);

  // 提取ROI，x,y,z范围
  roi_clip_.GetROI(in_cloud_ptr, clip_cloud_ptr,poses_);

  // 下采样，稀疏点云
  voxel_grid_filter_.downsample(clip_cloud_ptr, downsampled_cloud_ptr);

  // 地面分割;去掉平面 (效果太过了)
  patch_work_.estimate_ground(downsampled_cloud_ptr, ground_cloud_ptr, noground_cloud_ptr);

  // 聚类，将一定数量点云设为障碍物
  pcl::PointCloud<pcl::PointXYZI>::Ptr outCloudPtr(new pcl::PointCloud<pcl::PointXYZI>);
  std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> pointsVector;
  cluster_.segmentByDistance(downsampled_cloud_ptr, outCloudPtr, pointsVector);

  // 获取bounding_box信息
  autoware_msgs::CloudClusterArray inOutClusters;
  bounding_box_.getBoundingBox(header, pointsVector, inOutClusters);
  autoware_msgs::DetectedObjectArray detected_objects;

  // 发布障碍物信息
  publishDetectedObjects(inOutClusters, detected_objects);

  // 可视化
  visualization_msgs::MarkerArray visualize_markers;
  vdo_.visualizeDetectedObjs(detected_objects, visualize_markers);
  _pub_cluster_visualize_markers.publish(visualize_markers);

  // 发布topic
  publishCloud(&_pub_clip_cloud, in_sensor_cloud->header, clip_cloud_ptr);
  publishCloud(&_pub_down_cloud,in_sensor_cloud->header, downsampled_cloud_ptr);
  publishCloud(&_pub_noground_cloud, in_sensor_cloud->header, noground_cloud_ptr);
  publishCloud(&_pub_cluster_cloud, in_sensor_cloud->header, outCloudPtr);

}

void lidarObstacleDetection::publishDetectedObjects(
    const autoware_msgs::CloudClusterArray &in_clusters,
    autoware_msgs::DetectedObjectArray &detected_objects)
{
  int min_i{};
  double min_x{},max_x{},min_y{},max_y{},min_z{},max_z{};

  detected_objects.header = in_clusters.header;
  geometry_msgs::Pose min_pose;
  for (size_t i = 0; i < in_clusters.clusters.size(); i++)
  {
    autoware_msgs::DetectedObject detected_object;
    detected_object.header = in_clusters.header;
    detected_object.label = "unknown";
    detected_object.score = 1.;
    detected_object.space_frame = in_clusters.header.frame_id;
    detected_object.pose = in_clusters.clusters[i].bounding_box.pose;
    detected_object.dimensions = in_clusters.clusters[i].dimensions;
    detected_object.pointcloud = in_clusters.clusters[i].cloud;
    detected_object.convex_hull = in_clusters.clusters[i].convex_hull;
    detected_object.valid = true;
    detected_objects.objects.push_back(detected_object);

    if(detected_object.pose.position.x<in_clusters.clusters[min_i].bounding_box.pose.position.x)
      min_i=i;
  }
  for(auto point : in_clusters.clusters[min_i].convex_hull.polygon.points)
  {
    if(min_x==0){
      min_x=point.x;
      max_x=point.x;
      min_y=point.y;
      max_y=point.y;
      min_z=point.z;
      max_z=point.z;
    }//初始化
    min_x=(min_x<point.x)? min_x:point.x;
    max_x=(max_x>point.x)? max_x:point.x;
    min_y=(min_y<point.y)? min_y:point.y;
    max_y=(max_y>point.y)? max_y:point.y;
    min_z=point.z;
    max_z=(in_clusters.clusters[min_i].bounding_box.pose.position.z-point.z)*2;
  }
  min_pose=in_clusters.clusters[min_i].bounding_box.pose;
  _pub_detected_objects.publish(detected_objects);
  _pub_min_pose.publish(min_pose);
}