#include <ros/ros.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/search/kdtree.h>
#include <pcl/surface/gp3.h>
#include <pcl/surface/processing.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/visualization/pcl_visualizer.h>

boost::shared_ptr<pcl::visualization::PCLVisualizer> rgbVis (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud)
{
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
  viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
  return (viewer);
}
/*
void meshing(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cld_in, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cld_out)
{ 
*/ 
  /* refine the cloud 
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
  tree->setInputCloud(cld_in);
  pcl::MovingLeastSquares<pcl::PointXYZRGB, pcl::PointXYZRGB> mls;
  mls.setInputCloud(cld_in);
  mls.setSearchMethod(tree);
*/
  /* normal estimation 
  pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> n;
  pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::PointCloud<pcl::Normal>);
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
  tree->setInputCloud(cld_in);
  n.setInputCloud(cld_in);
  n.setSearchMethod(tree);
  n.setKSearch(20);
  n.compute(*normals);
*/
  /* 
  pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<PointNormal>);
  pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);
}
*/
int main(int argc, char** argv)
{
  ros::init (argc, argv, "test_pcl_generator");
  ros::NodeHandle nh;

  /* message */
  ros::Publisher pcl_pub = nh.advertise<sensor_msgs::PointCloud2>("/test_cloud",1);
  sensor_msgs::PointCloud2 pcl_msg;

  /* cloud */
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  
  point_cloud->width = 307200;
  point_cloud->height = 1;
  point_cloud->points.resize(point_cloud->width * point_cloud->height);
  
  for(unsigned int i = 0; i < point_cloud->points.size(); i++) {
    point_cloud->points[i].x = 0.2 * (rand() / (RAND_MAX + 1.0f ));
    point_cloud->points[i].y = 0.2 * (rand() / (RAND_MAX + 1.0f ));
    point_cloud->points[i].z = (point_cloud->points[i].x ) * (point_cloud->points[i].y );
    point_cloud->points[i].r = 255 * ((point_cloud->points[i].x )/ 0.1);
    point_cloud->points[i].g = 255 * ((point_cloud->points[i].y )/ 0.1);
    point_cloud->points[i].b = 255 * ((point_cloud->points[i].z )/ 0.1);
  }
 
  /* visualize */
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
  viewer = rgbVis(point_cloud);
  
  /* file I/O */
  // ply
  pcl::io::savePLYFileASCII ("/home/sys_er26/cloud_file/test_cloud.ply", *point_cloud);
  
  /* meshing */
  // Normal Esitimation
  pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> n;
  pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
  tree->setInputCloud (point_cloud);
  n.setInputCloud (point_cloud);
  n.setSearchMethod (tree);
  n.setKSearch (20);
  n.compute (*normals);
  
  // Concatenate the XYZ and normal fields*
  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
  pcl::concatenateFields (*point_cloud, *normals, *cloud_with_normals);
  //* cloud_with_normals = cloud + normals
  
  // Create search tree*
  pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointXYZRGBNormal>);
  tree2->setInputCloud (cloud_with_normals);  
  
  // Initialize objects
  pcl::GreedyProjectionTriangulation<pcl::PointXYZRGBNormal> gp3;
  pcl::PolygonMesh triangles;  
  
  // Set the maximum distance between connected points (maximum edge length)
  gp3.setSearchRadius (0.025);

  // Set typical values for the parameters
  gp3.setMu (2.5);
  gp3.setMaximumNearestNeighbors (100);
  gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
  gp3.setMinimumAngle(M_PI/18); // 10 degrees
  gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
  gp3.setNormalConsistency(false);

  // Get result
  gp3.setInputCloud (cloud_with_normals);
  gp3.setSearchMethod (tree2);
  gp3.reconstruct (triangles);

  // Additional vertex information
  //std::vector<int> parts = gp3.getPartIDs();
  //std::vector<int> states = gp3.getPointStates();
  
   pcl::io::savePLYFile("/home/sys_er26/cloud_file/test_mesh.ply", triangles);
//  pcl::io::saveOBJFile("/home/sys_er26/cloud_file/test_mesh.obj", triangles);
  // preprocess
//  pcl::CloudSurfaceProcessing<pcl::PointXYZRGB, pcl::PointXYZRGB>(*point_cloud);
//  pcl::io::savePLYFileASCII ("/home/sys_er26/cloud_file/test_cloud_pro.ply", *point_cloud);
  
 
  /* conversion */
  pcl::toROSMsg(*point_cloud,pcl_msg);
  pcl_msg.header.frame_id = "leap_work";
  
  /* publish */
  ros::Rate r(4);
  while (ros::ok())
  {
    pcl_pub.publish(pcl_msg);
    viewer->spinOnce (100);
    ros::spinOnce ();
    r.sleep ();
  }
  
  return 0;
}
