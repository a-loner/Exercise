/**
 * @ref http://eigen.tuxfamily.org/dox/GettingStarted.html
 */

#include <ros/ros.h>
#include <iostream>
#include <Eigen/Dense>

//using Eigen::MatrixXd;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "eigen_test");
  ros::NodeHandle nh;
  
  Eigen::MatrixXd m(2,2);
  m(0,0) = 3;
  m(1,0) = 2.5;
  m(0,1) = -1;
  m(1,1) = m(1,0) + m(0,1);
  std::cout << m << std::endl;  
  
  ros::spin();
  return 0;
}
