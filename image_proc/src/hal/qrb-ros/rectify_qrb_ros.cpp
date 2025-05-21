#include "image_proc/hal/qrb-ros/rectify_qrb_ros.hpp"

namespace image_proc {
namespace hal {
namespace qrb_ros {

RectifyQrbRos::RectifyQrbRos() : interpolation_(0) {}

void RectifyQrbRos::initialize(rclcpp::Node * node, int interpolation) {
  RCLCPP_INFO(node->get_logger(), "[DEBUG] initialize() called with interpolation: %d", interpolation);
  interpolation_ = interpolation;
}

subscription_types::SubscriptionHandle RectifyQrbRos::setupSubscriptions(
  rclcpp::Node * node,
  const std::string & topic,
  const std::function<void(
    const std::shared_ptr<void> &,
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr &)> & specialized_callback,
  const std::string & transport,
  const rclcpp::QoS & qos) {
  RCLCPP_INFO(node->get_logger(), "[DEBUG] setupSubscriptions() called for topic: %s with transport: %s", 
              topic.c_str(), transport.c_str());
  SubscriptionHandle handle;
  handle.specialized_sub = node->create_subscription<dmabuf_transport::type::Image>(
    topic, qos,
    [specialized_callback](std::shared_ptr<const dmabuf_transport::type::Image> msg) {
      std::shared_ptr<void> void_ptr(msg, const_cast<void*>(static_cast<const void*>(msg.get())));
      specialized_callback(void_ptr, nullptr);
    });
  return handle;
}

void RectifyQrbRos::imageCb(
  rclcpp::Node * node,
  const std::shared_ptr<void> & image_msg_void,
  const sensor_msgs::msg::CameraInfo::ConstSharedPtr & info_msg,
  image_geometry::PinholeCameraModel & model,
  int interpolation) {
  auto dmabuf_msg = std::static_pointer_cast<dmabuf_transport::type::Image>(image_msg_void);
  if (!dmabuf_msg) {
    RCLCPP_ERROR(node->get_logger(), "[QRB-ROS HAL] imageCb: Unsupported message type for QRB-ROS vendor");
    return;
  }
  (void)info_msg;
  (void)model;
  (void)interpolation;
  rclcpp::QoS qos{ 10 };
  qos.transient_local();
  node->create_publisher<dmabuf_transport::type::Image>("rectify_image", qos);
}

}  // namespace qrb_ros
}  // namespace hal
}  // namespace image_proc
