#include <mutex>
#include <memory>
#include <iostream>

#include <ros/ros.h>
#include <ros/console.h>
#include <pcl_ros/point_cloud.h>
#include <tf/transform_broadcaster.h>

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/approximate_voxel_grid.h>


pcl::PointCloud<pcl::PointXYZI>::Ptr globalmap;
ros::Publisher globalmap_pub;
std::string globalmap_pcd;
double downsample_resolution;

void cloud_init(){

  ROS_INFO("initializing started");
    
  // read globalmap from a pcd file
  
  globalmap.reset(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::io::loadPCDFile(globalmap_pcd, *globalmap);
  globalmap->header.frame_id = "map";

  // downsample globalmap
  
  boost::shared_ptr<pcl::ApproximateVoxelGrid<pcl::PointXYZI>> voxelgrid(new pcl::ApproximateVoxelGrid<pcl::PointXYZI>());
  voxelgrid->setLeafSize(downsample_resolution, downsample_resolution, downsample_resolution);
  voxelgrid->setInputCloud(globalmap);

  pcl::PointCloud<pcl::PointXYZI>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZI>());
  voxelgrid->filter(*filtered);

  globalmap = filtered;
    
}

int main(int argc, char** argv){

  ROS_INFO("Map load start");
  ros::init(argc, argv, "pcd_map_server");
  
  ros::NodeHandle nh;
  globalmap_pcd = nh.param<std::string>("globalmap_pcd", "/home/neubility001/catkin_ws/src/pcd_loader_ros/data/map.pcd");
  downsample_resolution = nh.param<double>("downsample_resolution", 0.1);
  //init cloud
  cloud_init();

  ROS_INFO("initializing done");
  // publish globalmap with "latched" publisher
  globalmap_pub = nh.advertise<sensor_msgs::PointCloud2>("/globalmap", 5, true);
  ROS_INFO("publishing started");
  globalmap_pub.publish(globalmap);
  ros::spin();

  return 0;
}