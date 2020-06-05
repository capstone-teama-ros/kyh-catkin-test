#include <cmath>

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

#include "line_finder.h"

class LidarListener
{
public:
  LidarListener()
  {
    line_finder_.setResolution(0.015);
    line_finder_.setThreshold(20);
    line_finder_.setTheta(1 * M_PI / 180);
  }

  void callback(const sensor_msgs::LaserScan::ConstPtr& message)
  {
    line_finder_.findLines(lines_, message);
    for (size_t i = 0; i < lines_.size(); ++i)
    {
      ROS_INFO("Line %lu: rho = %f [m], theta = %f [rad]", i, lines_[i][0], lines_[i][1]);
    }
  }

private:
  LineFinder line_finder_;
  std::vector<cv::Vec2f> lines_;
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
