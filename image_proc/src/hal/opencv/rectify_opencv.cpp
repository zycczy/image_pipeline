// Copyright (c) 2025 HAL Implementation Team
// All rights reserved.
//
// Software License Agreement (BSD License 2.0)

#include "image_proc/hal/opencv/rectify_opencv.hpp"
#include "tracetools_image_pipeline/tracetools.h"
#include <opencv2/imgproc.hpp>

namespace image_proc
{
namespace hal
{
namespace opencv
{

RectifyOpenCV::RectifyOpenCV()
: interpolation_(cv::INTER_LINEAR)
{
}

void RectifyOpenCV::initialize(rclcpp::Node * node, int interpolation)
{
  interpolation_ = interpolation;
  RCLCPP_INFO(node->get_logger(), "Initialized RectifyOpenCV HAL with interpolation: %d", interpolation);
}


void RectifyOpenCV::processImage(
  rclcpp::Node * node,
  const sensor_msgs::msg::Image::ConstSharedPtr & image_msg,
  const sensor_msgs::msg::CameraInfo::ConstSharedPtr & info_msg,
  image_geometry::PinholeCameraModel & model,
  const image_transport::Publisher & publisher,
  int interpolation)
{
  // Update the camera model
  model.fromCameraInfo(info_msg);

  // Create cv_bridge image to hold rectified image
  cv_bridge::CvImageConstPtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvShare(image_msg);
  } catch (const cv_bridge::Exception & e) {
    RCLCPP_ERROR(node->get_logger(), "cv_bridge exception: %s", e.what());
    return;
  }

  // Create cv_bridge image for output
  // Initialize rectified image with same encoding, size as image message
  // (mono, color, etc.)
  cv_bridge::CvImage rectified(
    image_msg->header, 
    image_msg->encoding,
    cv::Mat(cv_ptr->image.size(), cv_ptr->image.type()));

  // Rectify image using OpenCV
  model.rectifyImage(cv_ptr->image, rectified.image, interpolation);
  // Publish rectified image
  publisher.publish(rectified.toImageMsg());
}

}  // namespace opencv
}  // namespace hal
}  // namespace image_proc