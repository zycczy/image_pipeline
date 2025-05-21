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

image_transport::Publisher RectifyOpenCV::createPublisher(
  rclcpp::Node * node,
  const std::string & topic,
  const rmw_qos_profile_t & qos_profile,
  const rclcpp::PublisherOptions & options)
{
  // Create standard image_transport publisher
  (void)options;
  RCLCPP_DEBUG(node->get_logger(), "Creating standard image publisher for %s", topic.c_str());
  return image_transport::create_publisher(node, topic, qos_profile);
}

image_transport::Subscriber RectifyOpenCV::createImageSubscription(
  rclcpp::Node * node,
  const std::string & topic,
  const std::function<void(
    const sensor_msgs::msg::Image::ConstSharedPtr &)> & callback,
  const std::string & transport,
  const rmw_qos_profile_t & qos_profile)
{
  // Create standard image subscription using image_transport
  RCLCPP_DEBUG(node->get_logger(), "Creating standard image subscription for %s", topic.c_str());
  return image_transport::create_subscription(node, topic, callback, transport, qos_profile);
}

image_transport::CameraSubscriber RectifyOpenCV::createCameraSubscription(
  rclcpp::Node * node,
  const std::string & topic,
  const std::function<void(
    const sensor_msgs::msg::Image::ConstSharedPtr &,
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr &)> & callback,
  const std::string & transport,
  const rclcpp::QoS & qos)
{
  // Create standard camera subscription using image_transport
  RCLCPP_DEBUG(node->get_logger(), "Creating standard camera subscription for %s", topic.c_str());
  return image_transport::create_camera_subscription(
    node, topic, callback, transport, qos.get_rmw_qos_profile());
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