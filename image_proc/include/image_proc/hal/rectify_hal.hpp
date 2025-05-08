// Copyright (c) 2025 HAL Implementation Team
// All rights reserved.
//
// Software License Agreement (BSD License 2.0)

#ifndef IMAGE_PROC__HAL__RECTIFY_HAL_HPP_
#define IMAGE_PROC__HAL__RECTIFY_HAL_HPP_

#include <memory>
#include <string>
#include <functional>
#include <type_traits>
#include <variant>

#include "image_geometry/pinhole_camera_model.hpp"
#include "rclcpp/rclcpp.hpp"
#include "image_transport/image_transport.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace image_proc
{
namespace hal
{

// Forward declarations for subscription types
namespace subscription_types
{
  struct SubscriptionHandle
  {
    image_transport::CameraSubscriber camera_sub;
    rclcpp::SubscriptionBase::SharedPtr specialized_sub;  // For vendor-specific subscription
  };
}

class RectifyHAL
{
public:
  using SharedPtr = std::shared_ptr<RectifyHAL>;
  using SubscriptionHandle = subscription_types::SubscriptionHandle;
  
  RectifyHAL() = default;
  virtual ~RectifyHAL() = default;

  // 工厂方法创建 HAL 实现
  static SharedPtr create(
    const std::string & vendor_name,
    rclcpp::Node * node,
    int interpolation);

  // 初始化方法
  virtual void initialize(rclcpp::Node * node, int interpolation) = 0;

  // 统一方法设置所有订阅（标准和专用）
  virtual SubscriptionHandle setupSubscriptions(
    rclcpp::Node * node,
    const std::string & topic,
    const std::function<void(
      const std::shared_ptr<void> &,
      const sensor_msgs::msg::CameraInfo::ConstSharedPtr &)> & specialized_callback,
    const std::string & transport,
    const rclcpp::QoS & qos)
  {
    // 创建一个适配器回调，将std::shared_ptr<void>转换为sensor_msgs::msg::Image::ConstSharedPtr
    auto adapter_callback = 
      [specialized_callback](
        const sensor_msgs::msg::Image::ConstSharedPtr & image_msg,
        const sensor_msgs::msg::CameraInfo::ConstSharedPtr & info_msg) {
          specialized_callback(std::const_pointer_cast<void>(std::static_pointer_cast<const void>(image_msg)), info_msg);
        };

    // 创建标准相机订阅
    SubscriptionHandle handle;
    //create image_transport::create_camera_subscription and return handle
    handle.camera_sub = image_transport::create_camera_subscription(
      node, topic, adapter_callback, transport, qos.get_rmw_qos_profile());

    
    return handle;
  }
  
  // 图像回调处理 - 子类将特化此模板
  template<typename MsgT>
  void imageCb(
    rclcpp::Node * node,
    const typename MsgT::ConstSharedPtr & image_msg,
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr & info_msg,
    image_geometry::PinholeCameraModel & model,
    int interpolation)
  {
  }

  // 新增虚函数，统一入口，参数为void指针，由子类自行判断类型
  virtual void imageCb(
    rclcpp::Node * node,
    const std::shared_ptr<void> & image_msg_void,
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr & info_msg,
    image_geometry::PinholeCameraModel & model,
    int interpolation)
  {
    // 默认实现：警告未实现
    RCLCPP_WARN(node->get_logger(), "RectifyHAL::imageCb() not implemented for this vendor/type");
  }
};

}  // namespace hal
}  // namespace image_proc

#endif  // IMAGE_PROC__HAL__RECTIFY_HAL_HPP_