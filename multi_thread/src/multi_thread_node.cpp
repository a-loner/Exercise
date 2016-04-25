#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/console.h>
#include <std_msgs/String.h>
#include <string>

class Pub{
 private:
  ros::NodeHandle nh_;
  ros::Publisher pub_;
 public:
  Pub(ros::CallbackQueue& cq) {
    nh_.setCallbackQueue(&cq);
    pub_ = nh_.advertise < std_msgs::String > ("/multi_thread", 1); 
  }
  void pub(std::string str){
    ROS_INFO("send:%s", str.c_str());
    std_msgs::String msg_;
    msg_.data = str;
    pub_.publish(msg_);
  }
};

class Sub{
 private:
  ros::NodeHandle nh_;
  ros::Subscriber sub_;
  void Cb(const std_msgs::String& str){
    std::string s = str.data;
    ROS_INFO("received: %s",s.c_str());
  }
 public:
  Sub(ros::CallbackQueue& cq) {
    nh_.setCallbackQueue(&cq);
    sub_ = nh_.subscribe("/multi_thread", 1, &Sub::Cb, this);
  }
};

ros::CallbackQueue cqa;
ros::CallbackQueue cqb;

int main(int argc, char** argv) {
  ros::init(argc, argv, "multi_thread_node");
  Pub a(cqa);
  ros::AsyncSpinner spin_sub(1, &cqa);
  spin_sub.start();
  
  Sub b(cqb);
  ros::AsyncSpinner spin_pub(1, &cqb);
  spin_pub.start();  
  
  while(ros::ok()) {
    ros::Time begin = ros::Time::now();
    while(ros::Time::now() - begin < ros::Duration(0.5));
    a.pub(std::string("hello"));
  }  
  
  return 0;
}
