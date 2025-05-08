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

  void initialize(rclcpp::Node * node, int interpolation)
  {
    RCLCPP_INFO(node->get_logger(), "[DEBUG] initialize() called with interpolation: %d", interpolation);
    interpolation_ = interpolation;
  }

  // Set up subscriptions (both standard and DMA buffer)
  SubscriptionHandle setupSubscriptions(
    rclcpp::Node * node,
    const std::string & topic,
    const std::function<void(
      const std::shared_ptr<void> &,
      const sensor_msgs::msg::CameraInfo::ConstSharedPtr &)> & specialized_callback,
    const std::string & transport,
    const rclcpp::QoS & qos) 
    {
    RCLCPP_INFO(node->get_logger(), "[DEBUG] setupSubscriptions() called for topic: %s with transport: %s", 
                topic.c_str(), transport.c_str());
    SubscriptionHandle handle;
    handle.specialized_sub = node->create_subscription<dmabuf_transport::type::Image>(
      topic, qos,
      [specialized_callback](std::shared_ptr<const dmabuf_transport::type::Image> msg) {
        // Use aliasing constructor to safely cast to shared_ptr<void>
        std::shared_ptr<void> void_ptr(msg, const_cast<void*>(static_cast<const void*>(msg.get())));
        specialized_callback(void_ptr, nullptr);
      });
    return handle;
    }

  // 新增虚函数实现，统一入口
  void imageCb(
    rclcpp::Node * node,
    const std::shared_ptr<void> & image_msg_void,
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr & info_msg,
    image_geometry::PinholeCameraModel & model,
    int interpolation) override
  {
    // 判断类型是否为dmabuf_transport::type::Image
    auto dmabuf_msg = std::static_pointer_cast<dmabuf_transport::type::Image>(image_msg_void);
    if (!dmabuf_msg) {
      RCLCPP_ERROR(node->get_logger(), "[QRB-ROS HAL] imageCb: Unsupported message type for QRB-ROS vendor");
      return;
    }
    rclcpp::QoS qos{ 10 };
    qos.transient_local();
    node->create_publisher<dmabuf_transport::type::Image>("rectify_image", qos);
  }



private:
  int interpolation_;
};
}  // namespace qrb_ros
}  // namespace hal
}  // namespace image_proc

#endif  // IMAGE_PROC__HAL__QRB_ROS__RECTIFY_QRB_ROS_HPP_