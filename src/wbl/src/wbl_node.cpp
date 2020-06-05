#include <ros/ros.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "wbl");
  ros::NodeHandle n;

  ros::Rate sleep_duration(50);  // milliseconds

  while (ros::ok())
  {
    ros::spinOnce();
    sleep_duration.sleep();
  }
}
