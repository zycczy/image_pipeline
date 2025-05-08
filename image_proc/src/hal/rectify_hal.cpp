// Copyright (c) 2025 HAL Implementation Team
// All rights reserved.
//
// Software License Agreement (BSD License 2.0)

#include "image_proc/hal/rectify_hal.hpp"
#include "image_proc/hal/opencv/rectify_opencv.hpp"
#include "image_proc/hal/qrb-ros/rectify_qrb_ros.hpp"

namespace image_proc
{
namespace hal
{

RectifyHAL::SharedPtr RectifyHAL::create(
  const std::string & vendor_name,
  rclcpp::Node * node,
  int interpolation)
{
  RectifyHAL::SharedPtr hal;

  if (vendor_name == "qrb-ros") {
    RCLCPP_INFO(node->get_logger(), "Creating QRB-ROS Rectify HAL");
    hal = std::make_shared<qrb_ros::RectifyQrbRos>();
  } else {
    // Default to OpenCV implementation
    RCLCPP_INFO(node->get_logger(), "Creating OpenCV Rectify HAL");
    hal = std::make_shared<opencv::RectifyOpenCV>();
  }

  hal->initialize(node, interpolation);
  return hal;
}

}  // namespace hal
}  // namespace image_proc