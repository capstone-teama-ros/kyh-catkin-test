#ifndef WBL_LINE_FINDER_H
#define WBL_LINE_FINDER_H

#include <vector>

#include <sensor_msgs/LaserScan.h>
#include <opencv2/core/core.hpp>

class LineFinder
{
public:
  void setBufferSize(int buffer_size)
  {
    buffer_size_ = buffer_size;
  }

  void setThreshold(int threshold)
  {
    threshold_ = threshold;
  }

  void setRho(double rho)
  {
    rho_ = rho;
  }

  void setTheta(double theta)
  {
    theta_ = theta;
  }

  /**
   * Applies Standard Hough Line Transform to an image to detect lines.
   * @param lines Output vector of (rho, theta) for each line found.
   *    rho is measured in meters.
   *    theta is measured in radians.
   * @param data  LaserScan sensor data
   */
  void findLines(std::vector<cv::Vec2f>& lines, const sensor_msgs::LaserScan::ConstPtr& data);

private:
  cv::Mat buffer_;  ///< Buffer image to use when converting a series of points to an 2D image.

  int buffer_size_ = 300;  ///< Width and height of buffer image in pixels.
  int threshold_ = 10;     ///< Accumulator threshold (minimum vote).
  double rho_ = 0.01;      ///< Distance resolution of the accumulator in meters.
  double theta_ = 1;       ///< Distance resolution of the accumulator in radians.
};

#endif
