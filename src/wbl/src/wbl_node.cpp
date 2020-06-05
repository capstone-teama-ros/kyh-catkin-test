#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

class LidarListener
{
public:
  void callback(const sensor_msgs::LaserScan::ConstPtr& message)
  {
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "wbl");
  ros::NodeHandle n;

  LidarListener lidar_listener;
  ros::Subscriber lidar_subscriber = n.subscribe("/scan", 1000, &LidarListener::callback, &lidar_listener);

  ros::Rate sleep_duration(50);  // milliseconds

  while (ros::ok())
  {
    ros::spinOnce();
    sleep_duration.sleep();
  }
}
