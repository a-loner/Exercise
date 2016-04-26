# exercise
a collection of naive programing sample

## eigen_test
demonstrates how to use Eigen Library with ROS indigo

## pcl_file_io_test
demonstrates how to make conversion between point cloud message and file

## multi_thread
example for multi-thread programing in ROS

at the **beginning of file:**
```cpp
#include <ros/callback_queue.h> 
```
in **global range**
```cpp
ros::CallbackQueue cq; // Define a callback queue.
```
in **main()** or **object constructor**
```cpp
ros::NodeHandle nh;
nh.setCallbackQueue(&cq); // Assigh cq to specific nh
```
in **main()**
```cpp
ros::AsyncSpinner aspin(1, &cq); // Define a spinner and bind it with cq
aspin.start();                   // Start the spinner 
```
