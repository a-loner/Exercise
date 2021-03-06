/**
 * \file pcl_file_io_test.cpp
 * 
 * FILE DESCRIPTION
 * test the conversion between point cloud message and file
 *
 * \author a-loner
 */

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/pcl_macros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>

namespace PclFileIoTest
{
class CloudManager
{
private:
  ros::NodeHandle nh_;

  /* receive point cloud */
  ros::Subscriber pcl_sub_;
  void pclCallback(const sensor_msgs::PointCloud2& msg);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_temp_;

public:
  CloudManager();
  ~CloudManager();
};

CloudManager::CloudManager():
  cloud_temp_(new pcl::PointCloud<pcl::PointXYZRGB>)
{
  /* receive point cloud */
  pcl_sub_ = nh_.subscribe("/test_cloud", 2, &CloudManager::pclCallback, this);
}

CloudManager::~CloudManager()
{

}

void CloudManager::pclCallback(const sensor_msgs::PointCloud2& msg)
{
  ROS_INFO("received point cloud message");
  //pcl::fromROSMsg(msg, cloud_temp_);
  
}

} // namespace

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pcl_file_io_test");
  PclFileIoTest::CloudManager cm;
  ros::spin();
  return 0;
}
