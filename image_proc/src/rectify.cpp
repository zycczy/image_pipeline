// Copyright 2008, 2019 Willow Garage, Inc., Andreas Klintberg, Joshua Whitley
// All rights reserved.
//
// Software License Agreement (BSD License 2.0)
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
// * Redistributions of source code must retain the above copyright
//   notice, this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above
//   copyright notice, this list of conditions and the following
//   disclaimer in the documentation and/or other materials provided
//   with the distribution.
// * Neither the name of {copyright_holder} nor the names of its
//   contributors may be used to endorse or promote products derived
//   from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include <functional>
#include <mutex>
#include <string>

#include "cv_bridge/cv_bridge.hpp"
#include "tracetools_image_pipeline/tracetools.h"

#include <image_proc/rectify.hpp>
#include <image_proc/utils.hpp>
#include <image_proc/hal/rectify_hal.hpp>
#include "dmabuf_transport/type/image.hpp"

#include <image_transport/image_transport.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace image_proc
{

RectifyNode::RectifyNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("RectifyNode", options)
{
  // TransportHints does not actually declare the parameter
  this->declare_parameter<std::string>("image_transport", "raw");

  // For compressed topics to remap appropriately, we need to pass a
  // fully expanded and remapped topic name to image_transport
  auto node_base = this->get_node_base_interface();
  image_topic_ = node_base->resolve_topic_or_service_name("image", false);

  queue_size_ = this->declare_parameter("queue_size", 5);
  interpolation_ = this->declare_parameter("interpolation", 1);
  vendor_name_ = this->declare_parameter("vendor", "default");

  RCLCPP_INFO(this->get_logger(), "Initializing Rectify node with vendor: %s", vendor_name_.c_str());

  // Create the HAL implementation based on vendor name
  hal_ = hal::RectifyHAL::create(vendor_name_, this, interpolation_);
  if (!hal_) {
    RCLCPP_ERROR(this->get_logger(), "Failed to create HAL implementation for vendor: %s", vendor_name_.c_str());
    return;
  }
  
  
  // Set up lazy subscriptions (activated only when someone subscribes to our publisher)
  auto qos_profile = getTopicQosProfile(this, image_topic_);
  // Convert rmw_qos_profile_s to rclcpp::QoS
  rclcpp::QoS qos = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(qos_profile), qos_profile);
  
  subscription_handle_ = hal::RectifyHAL::SubscriptionHandle();

  
  // Start with subscriptions active by default
  setup_lazy_subscriptions(qos);
}

void RectifyNode::setup_lazy_subscriptions(const rclcpp::QoS& qos_profile)
{
  // Skip if subscriptions are already set up
  if (subscription_handle_.camera_sub || subscription_handle_.specialized_sub) {
    RCLCPP_DEBUG(this->get_logger(), "Subscriptions already set up, skipping");
    return;
  }
  
  RCLCPP_INFO(this->get_logger(), "Setting up subscriptions for %s", image_topic_.c_str());
  
  // Create camera info subscription for specialized message types if needed

  auto camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
    "camera_info", 10,
    [this](const sensor_msgs::msg::CameraInfo::ConstSharedPtr & msg) {
      latest_camera_info_ = msg;
    });
  
  // Create transport hints for image transport
  image_transport::TransportHints hints(this);
  
  // Use the HAL's unified subscription setup method
  // This will create both standard and specialized subscriptions as needed
  subscription_handle_ = hal_->setupSubscriptions(
    this,
    image_topic_,
    // callback (gets converted to specific type inside HAL)
    std::bind(&RectifyNode::ImageCb, this,
              std::placeholders::_1, std::placeholders::_2),
    hints.getTransport(),
    qos_profile);
    
  if (!subscription_handle_.camera_sub && !subscription_handle_.specialized_sub) {
    RCLCPP_ERROR(this->get_logger(), "Failed to set up any subscriptions!");
  } else {
    RCLCPP_INFO(this->get_logger(), "Successfully set up subscriptions");
  }
}

sensor_msgs::msg::CameraInfo::ConstSharedPtr RectifyNode::get_latest_camera_info()
{
  // This is a helper method to get the latest camera info for specialized message types
  return latest_camera_info_;
}

void RectifyNode::ImageCb(
  const std::shared_ptr<void> & image_msg_void,
  const sensor_msgs::msg::CameraInfo::ConstSharedPtr & info_msg)
{
  // If no info message is provided, try to use the latest one
  auto actual_info_msg = info_msg ? info_msg : get_latest_camera_info();
  
  if (!actual_info_msg) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), 
      *this->get_clock(), 
      1000,  // Throttle to once per second
      "No camera info available for specialized message");
    return;
  }

  // Delegate message handling to HAL, which will handle type and vendor specifics
  hal_->imageCb(this, image_msg_void, actual_info_msg, model_, interpolation_);
}

}  // namespace image_proc

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(image_proc::RectifyNode)
