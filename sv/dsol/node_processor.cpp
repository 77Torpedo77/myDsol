#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <rosgraph_msgs/Clock.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/String.h>
#include <visualization_msgs/Marker.h>
#include "geometry_msgs/PoseStamped.h"


#include <iostream>

namespace sv::dsol {
void nodeDataCallback(const rosgraph_msgs::Clock::ConstPtr& msg) {
  std::cout << "Received /clock message: " << msg->clock << std::endl;
}

void gtCallback(const std_msgs::String::ConstPtr& msg) {
  std::cout << "Received /gt message: " << msg->data << std::endl;
}

void kfCallback(const std_msgs::String::ConstPtr& msg) {
  std::cout << "Received /kf message: " << msg->data << std::endl;
}

void poseOdomCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
  std::cout << "Received /dsol_data/pose_odom message" << std::endl;
  std::cout << "Position: [x: " << msg->pose.position.x
            << ", y: " << msg->pose.position.y
            << ", z: " << msg->pose.position.z << "]" << std::endl;
  std::cout << "Orientation: [x: " << msg->pose.orientation.x
            << ", y: " << msg->pose.orientation.y
            << ", z: " << msg->pose.orientation.z
            << ", w: " << msg->pose.orientation.w << "]" << std::endl;
}

void pointsCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
  std::cout << "Received /points message with width: " << msg->width
            << " and height: " << msg->height << std::endl;
}

void poseArrayCallback(const geometry_msgs::PoseArray::ConstPtr& msg) {
  std::cout << "Received /pose_array message with " << msg->poses.size()
            << " poses" << std::endl;
}

void alignMarkerCallback(const visualization_msgs::Marker::ConstPtr& msg) {
  std::cout << "Received /align_marker message with id: " << msg->id
            << std::endl;
}
}  // namespace sv::dsol

int main(int argc, char** argv) {
  ros::init(argc, argv, "node_processor");
  ros::NodeHandle n;

  std::cout << "Subscribing to topics..." << std::endl;

  // ros::Subscriber clock_sub = n.subscribe("/clock", 1000, sv::dsol::nodeDataCallback);
  std::cout << "Subscribed to /clock" << std::endl;
  // ros::Subscriber gt_sub = n.subscribe("/dsol_data/gt", 1000, sv::dsol::gtCallback);
  std::cout << "Subscribed to /gt" << std::endl;
  // ros::Subscriber kf_sub = n.subscribe("kf", 1000, sv::dsol::kfCallback);
  std::cout << "Subscribed to /kf" << std::endl;
  ros::Subscriber odom_sub = n.subscribe("/dsol_data/pose_odom", 1000, sv::dsol::poseOdomCallback);
  std::cout << "Subscribed to /odom" << std::endl;
  ros::Subscriber points_sub = n.subscribe("points", 1000, sv::dsol::pointsCallback);
  std::cout << "Subscribed to /points" << std::endl;
  ros::Subscriber pose_array_sub = n.subscribe("pose_array", 1000, sv::dsol::poseArrayCallback);
  std::cout << "Subscribed to /pose_array" << std::endl;
  ros::Subscriber align_marker_sub = n.subscribe("align_marker", 1000, sv::dsol::alignMarkerCallback);
  std::cout << "Subscribed to /align_marker" << std::endl;

  std::cout << "Subscribed to all topics. Waiting for messages====" << std::endl;

  ros::spin();
  return 0;
}
