# exercise
a collection of naive programing sample

## eigen_test
demonstrates how to use Eigen Library with ROS indigo

## pcl_file_io_test
demonstrates how to make conversion between point cloud message and file

## multi_thread
example for multi-thread programing in ROS

0. include header file *ros/callback_queue.h* **at the beginning of file**
```
#include <ros/callback_queue.h>
```
1. Define a **callback queue** *cq*
```
ros::CallbackQueue cq;
```
2. Assigh the **callback queue** *cq* to specific **NodeHandle** *nh*
```
nh.setCallbackQueue(&cq);
```
3. Define a **asyncspinner** *aspin* and bind it with *cq*
```
ros::AsyncSpinner aspin(1, &cq);
```
4. start the spinner 
```
aspin.start()
```
