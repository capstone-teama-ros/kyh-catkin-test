#include "line_finder.h"

#include <algorithm>
#include <cmath>

#include <ros/ros.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

void LineFinder::findLines(std::vector<cv::Vec2f>& lines, const sensor_msgs::LaserScan::ConstPtr& data)
{
  // Fill the matrix with zeros
  buffer_ = cv::Mat::zeros(buffer_size_, buffer_size_, CV_8UC1);

  // Create a rect for checking if a point lies within the image bounds
  const cv::Rect imageRect(cv::Point(), buffer_.size());
  // Compute the resolution (number of pixels per meter)
  // Add 5% margin at the edges of the buffer to ensure that no points are clipped
  double pixels_per_meter = 0.95 * buffer_size_ / 2 / std::max<double>(0.1, data->range_max);

  // Build a binary (black-and-white) image from LIDAR data
  for (size_t i = 0; i < data->ranges.size(); ++i)
  {
    float range = data->ranges[i];
    if (range < data->range_min || range > data->range_max)
      continue;

    float angle = data->angle_min + data->angle_increment * i;

    // Transform polar coordinates (origin at robot) to cartesian (origin at left top of buffer)
    float obstacle_x = range * std::cos(angle);
    float obstacle_y = range * std::sin(angle);
    int cx = cvRound(buffer_.cols / 2.0 + obstacle_x * pixels_per_meter);
    int cy = cvRound(buffer_.rows / 2.0 + obstacle_y * pixels_per_meter);

    if (imageRect.contains({ cx, cy }))
      buffer_.at<unsigned char>({ cx, cy }) = 255;
  }

  lines.empty();
  // Standard Hough Line Transform
  cv::HoughLines(buffer_, lines, rho_ * pixels_per_meter, theta_, threshold_, 0, 0);

  // Translate line from buffer coordinates to robot coordinates
  for (auto& line : lines)
  {
    double rho = line[0];
    double theta = line[1];
    auto cos_theta = std::cos(theta);
    auto sin_theta = std::sin(theta);

    // Direction unit vector of the line
    cv::Vec2f direction(sin_theta, -cos_theta);
    // Point on the line nearest to the buffer origin (in buffer coordinates)
    cv::Point2f c(rho * cos_theta, rho * sin_theta);
    // Point on the line nearest to the buffer origin (in robot coordinates)
    auto c_robot = c - cv::Point2f(buffer_.cols / 2.0, buffer_.rows / 2.0);

    // Find the point closest to the new origin (robot)
    // (Note: the magnitude of the direction vector is 1, so no division needed)
    auto r = c_robot - cv::Point2f(c_robot.dot(direction) * direction);

    // Convert back to (rho, theta) format (in robot coordinates)
    auto rho_new = std::sqrt(r.x * r.x + r.y * r.y);
    auto theta_new = std::atan2(r.y, r.x);

    // Replace the old values with the new ones
    // Also, convert the unit of rho from pixels to meters
    line[0] = rho_new / pixels_per_meter;
    line[1] = theta_new;
  }
}
