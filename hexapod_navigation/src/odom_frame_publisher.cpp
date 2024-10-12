#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/Quaternion.h"
#include <tf/transform_broadcaster.h>

// This functions get called when a new odometry message gets to the subscriber
// It automatically gets the odometry message as a parameter
// It prints out various parts of the message
void odom_callback(const nav_msgs::Odometry::ConstPtr &msg) {

  static tf::TransformBroadcaster broadcaster;    
  geometry_msgs::TransformStamped t;
  
  ROS_INFO("Frame ID: %s", msg->header.frame_id.c_str());
  ROS_INFO("Child Frame ID: %s", msg->child_frame_id.c_str());
  ROS_INFO("X: %f", msg->pose.pose.position.x);
  ROS_INFO("Y: %f", msg->pose.pose.position.y);
  
  t.header.frame_id = msg->header.frame_id;
  t.child_frame_id = msg->child_frame_id;
  t.transform.translation.x =  msg->pose.pose.position.x;
  t.transform.translation.y =  msg->pose.pose.position.y;
  // t.transform.translation.z = 0.0;
  t.transform.translation.z =  msg->pose.pose.position.z;

  t.transform.rotation = msg->pose.pose.orientation;
  t.header.stamp = msg->header.stamp;
  
  broadcaster.sendTransform(t);
  ROS_INFO("Broadcasting %s TF frame", t.header.frame_id.c_str());

}

int main(int argc, char **argv){
  ros::init(argc, argv, "odom_frame_publisher");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("gazebo_odom", 1000, odom_callback);
  ros::spin();

  return 0;
}
