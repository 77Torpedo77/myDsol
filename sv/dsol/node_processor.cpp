#include <geometry_msgs/PoseArray.h>

#include <ros/ros.h>

#include <std_msgs/String.h>


namespace sv::dsol {
void chatterCallback(const geometry_msgs::PoseArray::ConstPtr& msg)
{
  for (const auto& pose : msg->poses) {
    ROS_INFO("Pose position: [x: %f, y: %f, z: %f]", pose.position.x, pose.position.y, pose.position.z);
    ROS_INFO("Pose orientation: [x: %f, y: %f, z: %f, w: %f]", pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
  }
}
}


 
int main(int argc, char **argv)
{
    ros::init(argc, argv, "listener");
    ros::NodeHandle n("~");
    ros::Subscriber sub = n.subscribe("chatter", 1000, sv::dsol::chatterCallback);
 
    /**
     * ros::spin() 将会进入循环， 一直调用回调函数chatterCallback(),每次调用1000个数据。
     * 当用户输入Ctrl+C或者ROS主进程关闭时退出，
     */
    ros::spin();
    return 0;
}