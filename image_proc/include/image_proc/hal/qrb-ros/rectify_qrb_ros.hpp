// Copyright (c) 2025 HAL Implementation Team
// All rights reserved.
//
// Software License Agreement (BSD License 2.0)

#ifndef IMAGE_PROC__HAL__QRB_ROS__RECTIFY_QRB_ROS_HPP_
#define IMAGE_PROC__HAL__QRB_ROS__RECTIFY_QRB_ROS_HPP_

#include "image_proc/hal/rectify_hal.hpp"
#include "dmabuf_transport/type/image.hpp"
#include "cv_bridge/cv_bridge.hpp"
#include <type_traits>

namespace image_proc
{
namespace hal
{
namespace qrb_ros
{

class RectifyQrbRos : public RectifyHAL
{
public:
  RectifyQrbRos();
  virtual ~RectifyQrbRos() = default;

  void initialize(rclcpp::Node * node, int interpolation);

  // Set up subscriptions (both standard and DMA buffer)
  SubscriptionHandle setupSubscriptions(
    rclcpp::Node * node,
    const std::string & topic,
    const std::function<void(
      const std::shared_ptr<void> &,
      const sensor_msgs::msg::CameraInfo::ConstSharedPtr &)> & specialized_callback,
    const std::string & transport,
    const rclcpp::QoS & qos);

  // 新增虚函数实现，统一入口
  void imageCb(
    rclcpp::Node * node,
    const std::shared_ptr<void> & image_msg_void,
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr & info_msg,
    image_geometry::PinholeCameraModel & model,
    int interpolation) override;

private:
  int interpolation_;
};
}  // namespace qrb_ros
}  // namespace hal
}  // namespace image_proc

#endif  // IMAGE_PROC__HAL__QRB_ROS__RECTIFY_QRB_ROS_HPP_