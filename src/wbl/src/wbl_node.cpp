#include <cmath>

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "line_finder.h"

class LidarListener
{
public:
  LidarListener()
  {
    line_finder_.setThreshold(15);
    line_finder_.setRho(0.015);
    line_finder_.setTheta(2 * M_PI / 180);
    line_finder_.setBufferSize(500);
  }

  void callback(const sensor_msgs::LaserScan::ConstPtr& message)
  {
    line_finder_.findLines(lines_, message);

    image_ = cv::Mat::zeros(image_size_, image_size_, CV_8UC3);

    // Draw LIDAR points
    for (size_t i = 0; i < message->ranges.size(); ++i)
    {
      float range = message->ranges[i];
      if (range < message->range_min || range > message->range_max)
        continue;

      float angle = message->angle_min + message->angle_increment * i;

      // Transform polar coordinates (origin at robot) to cartesian (origin at left top of image)
      float point_x = range * std::cos(angle);
      float point_y = range * std::sin(angle);
      int cx = cvRound(image_.cols / 2.0 + point_x * pixels_per_meter);
      int cy = cvRound(image_.rows / 2.0 + point_y * pixels_per_meter);

      const int DOT_SIZE = 2;
      cv::Rect pointRect(cx - DOT_SIZE, cy - DOT_SIZE, 2 * DOT_SIZE, 2 * DOT_SIZE);
      cv::rectangle(image_, pointRect, cv::Scalar(255, 255, 0), CV_FILLED);
    }

    // Draw lines found
    for (auto& line : lines_)
    {
      auto rho = line[0];
      auto theta = line[1];
      auto cos_theta = std::cos(theta);
      auto sin_theta = std::sin(theta);

      auto cx = rho * cos_theta * pixels_per_meter + image_.cols / 2.0;
      auto cy = rho * sin_theta * pixels_per_meter + image_.rows / 2.0;

      // Create two distant points on the line
      int ax = cvRound(cx + 100 * image_.cols * sin_theta);
      int ay = cvRound(cy - 100 * image_.rows * cos_theta);
      int bx = cvRound(cx - 100 * image_.cols * sin_theta);
      int by = cvRound(cy + 100 * image_.rows * cos_theta);

      cv::line(image_, { ax, ay }, { bx, by }, cv::Scalar(0, 255, 255));
    }

    cv::imshow("Wall Detection", image_);
    cv::waitKey(100);
  }

private:
  LineFinder line_finder_;
  std::vector<cv::Vec2f> lines_;
  int image_size_ = 600;
  cv::Mat image_;
  double pixels_per_meter = 80;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "wbl");
  ros::NodeHandle n;

  LidarListener lidar_listener;
  ros::Subscriber lidar_subscriber = n.subscribe("/scan", 1000, &LidarListener::callback, &lidar_listener);

  ros::spin();
}
