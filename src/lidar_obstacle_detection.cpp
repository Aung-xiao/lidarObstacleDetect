#include "lidar_obstacle_detection.h"
#include <utility>

#define pi 3.14

int64_t gtm()
{
  struct timeval tm{};
  gettimeofday(&tm, nullptr);
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

Eigen::Matrix<double, 4, 4> T_param(double theta, double d, double a,
                                                            double alpha)
{
  Eigen::Matrix<double, 4, 4> T;
  T<<cos(theta), -sin(theta)*cos(alpha), sin(theta)*sin(alpha) ,  a*cos(theta),
      sin(theta), cos(theta)*cos(alpha) , -cos(theta)*sin(alpha), a*sin(theta),
      0         , sin(alpha)            , cos(alpha)            , d           ,
      0         , 0                     , 0                     , 1           ;
  return T;
}


lidarObstacleDetection::lidarObstacleDetection(ros::NodeHandle nh, const ros::NodeHandle& pnh)
    : roi_clip_(nh, pnh), voxel_grid_filter_(nh, pnh), cluster_(nh, pnh), bounding_box_(pnh)
{
  ros::Subscriber lidar_sub = nh.subscribe("/livox_horizon_points", 1, &lidarObstacleDetection::ClusterCallback, this);
  ros::Subscriber arm_sub=nh.subscribe("/livox_horizon_points", 1, &lidarObstacleDetection::ArmThetaCallback, this);
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

void lidarObstacleDetection::ArmThetaCallback(const sensor_msgs::PointCloud2ConstPtr &in_sensor_cloud)
{
  tfBroadcaster();
  std::vector<double> a={0, -0.42500, -0.39225, 0, 0, 0};
  std::vector<double> d={0.089159, 0, 0, 0.10915, 0.09465, 0.08230};
  std::vector<double> alpha={pi/2, 0, 0, pi/2, -pi/2, 0};
  std::vector<double> theta={pi/2, 0, 0, pi/2, -pi/2, 0};
  poses_.clear();

  Eigen::Matrix<double, 4, 4>T01= T_param(theta[0],d[0],a[0],alpha[0]);
  Eigen::Matrix<double, 4, 4>T12= T_param(theta[1],d[1],a[1],alpha[1]);
  Eigen::Matrix<double, 4, 4>T23= T_param(theta[2],d[2],a[2],alpha[2]);
  Eigen::Matrix<double, 4, 4>T34= T_param(theta[3],d[3],a[3],alpha[3]);
  Eigen::Matrix<double, 4, 4>T45= T_param(theta[4],d[4],a[4],alpha[4]);
  Eigen::Matrix<double, 4, 4>T56= T_param(theta[5],d[5],a[5],alpha[5]);

  Eigen::Matrix<double, 4, 4>T06=T01*T12*T23*T34*T45*T56;
  tf2::Vector3 pose={T06(0,3),T06(1,3),T06(2,3)};
  poses_.push_back(pose);

}

void lidarObstacleDetection::tfBroadcaster()
{
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin( tf::Vector3(10,10, 10.0) );
  tf::Quaternion q;
  q.setRPY(0, 0, 0);
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", "test"));
}
