// Copyright (c) 2025 HAL Implementation Team
// All rights reserved.
//
// Software License Agreement (BSD License 2.0)

#ifndef IMAGE_PROC__HAL__OPENCV__RECTIFY_OPENCV_HPP_
#define IMAGE_PROC__HAL__OPENCV__RECTIFY_OPENCV_HPP_

#include "image_proc/hal/rectify_hal.hpp"
#include "cv_bridge/cv_bridge.hpp"
#include <type_traits>

namespace image_proc
{
namespace hal
{
namespace opencv
{

class RectifyOpenCV : public RectifyHAL
{
public:
  RectifyOpenCV();
  virtual ~RectifyOpenCV() = default;

  void initialize(rclcpp::Node * node, int interpolation) override;

  image_transport::Publisher createPublisher(
    rclcpp::Node * node,
    const std::string & topic,
    const rmw_qos_profile_t & qos_profile,
    const rclcpp::PublisherOptions & options);

  image_transport::Subscriber createImageSubscription(
    rclcpp::Node * node,
    const std::string & topic,
    const std::function<void(
      const sensor_msgs::msg::Image::ConstSharedPtr &)> & callback,
    const std::string & transport,
    const rmw_qos_profile_t & qos_profile);

  image_transport::CameraSubscriber createCameraSubscription(
    rclcpp::Node * node,
    const std::string & topic,
    const std::function<void(
      const sensor_msgs::msg::Image::ConstSharedPtr &,
      const sensor_msgs::msg::CameraInfo::ConstSharedPtr &)> & callback,
    const std::string & transport,
    const rclcpp::QoS & qos);

  // Process image implementation (not an override)
  void processImage(
    rclcpp::Node * node,
    const sensor_msgs::msg::Image::ConstSharedPtr & image_msg,
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr & info_msg,
    image_geometry::PinholeCameraModel & model,
    const image_transport::Publisher & publisher,
    int interpolation);

  // Template specialization for standard ROS Image
  template<typename MsgT>
  void imageCb(
    rclcpp::Node * node,
    const typename MsgT::ConstSharedPtr & image_msg,
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr & info_msg,
    image_geometry::PinholeCameraModel & model,
    int interpolation)
  {
    if (!image_msg || !info_msg) {
      RCLCPP_ERROR(node->get_logger(), "Invalid image or camera info message");
      return;
    }

    RCLCPP_DEBUG(node->get_logger(), "OpenCV HAL: Processing image");
    
    // Create publisher if needed
    auto pub = image_transport::create_publisher(node, "image_rect");
    
    // Process the image
    if constexpr (std::is_same_v<MsgT, sensor_msgs::msg::Image>) {
      processImage(node, std::static_pointer_cast<const sensor_msgs::msg::Image>(image_msg), 
                  info_msg, model, pub, interpolation);
    } else {
      RCLCPP_WARN(
        node->get_logger(),
        "Unhandled message type in OpenCV HAL imageCb - no processing performed");
    }
  }

  // 新增虚函数实现，统一入口
  void imageCb(
    rclcpp::Node * node,
    const std::shared_ptr<void> & image_msg_void,
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr & info_msg,
    image_geometry::PinholeCameraModel & model,
    int interpolation) override
  {
    // 用 static_pointer_cast 替换 dynamic_pointer_cast
    std::shared_ptr<sensor_msgs::msg::Image> image_msg;
    try {
      image_msg = std::static_pointer_cast<sensor_msgs::msg::Image>(image_msg_void);
    } catch (const std::bad_cast &e) {
      RCLCPP_ERROR(node->get_logger(), "[OpenCV HAL] imageCb: Bad cast for image_msg_void: %s", e.what());
      return;
    }
    if (!image_msg) {
      RCLCPP_ERROR(node->get_logger(), "[OpenCV HAL] imageCb: image_msg_void is not a sensor_msgs::msg::Image");
      return;
    }
    // 创建publisher
    auto pub = image_transport::create_publisher(node, "image_rect");
    // 调用原有处理逻辑
    processImage(node, image_msg, info_msg, model, pub, interpolation);
  }


private:
  int interpolation_;
};

}  // namespace opencv
}  // namespace hal
}  // namespace image_proc

#endif  // IMAGE_PROC__HAL__OPENCV__RECTIFY_OPENCV_HPP_