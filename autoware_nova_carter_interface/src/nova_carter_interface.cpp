// Copyright 2025 The Autoware Foundation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "nova_carter_interface.hpp"
#include <cmath>

namespace autoware::nova_carter_interface
{

NovaCarterInterface::NovaCarterInterface(const rclcpp::NodeOptions & options)
: Node("nova_carter_interface", options)
{
  // Get Parameters
  base_frame_id_ = this->declare_parameter<std::string>("base_frame_id");
  maximum_linear_velocity_ = this->declare_parameter<double>("maximum_linear_velocity");
  maximum_angular_velocity_ = this->declare_parameter<double>("maximum_angular_velocity");
  // TODO(mitsudome-r): Get from vehicle info utils
  virtual_wheel_base_ = this->declare_parameter<double>("wheel_base");

  // wheel base cannot be zero
  if (std::abs(virtual_wheel_base_) < 1e-6) {
    throw std::runtime_error("Wheel base is zero or too small");
  }

  // Initialize subscribers
  control_cmd_sub_ = this->create_subscription<ControlMsg>(
    "control_cmd", 10, std::bind(&NovaCarterInterface::control_cmd_callback, this, std::placeholders::_1));

  odom_sub_ = this->create_subscription<OdometryMsg>(
    "odom", 10, std::bind(&NovaCarterInterface::odometry_callback, this, std::placeholders::_1));

  // Initialize publishers
  twist_pub_ = this->create_publisher<TwistMsg>("cmd_vel", 10);
  vehicle_velocity_pub_ = this->create_publisher<VelocityReportMsg>("vehicle_velocity_report", 10);
  steering_status_pub_ = this->create_publisher<SteeringReportMsg>("steering_status_report", 10);
}

/**
 * @brief Callback function for odometry messages from Nova Carter
 * @param [in] msg Shared pointer to odometry message containing velocity and position data
 * @details Processes odometry data to:
 *   - Calculate steering angle from velocities using bicycle model
 *   - Publish steering status report with calculated angle
 *   - Publish velocity report with linear velocity
 */
void NovaCarterInterface::odometry_callback(const OdometryMsg::ConstSharedPtr msg)
{
  RCLCPP_DEBUG(this->get_logger(), "Received odometry: velocity = %f, angular_velocity = %f",
              msg->twist.twist.linear.x, msg->twist.twist.angular.z);

  const double linear_velocity = msg->twist.twist.linear.x;
  const double angular_velocity = msg->twist.twist.angular.z;
  const double steering_angle = std::atan2(angular_velocity * virtual_wheel_base_, linear_velocity);

  // Publish steering status
  auto steering_report_msg = SteeringReportMsg();
  steering_report_msg.stamp = this->now();
  steering_report_msg.steering_tire_angle = steering_angle;
  steering_status_pub_->publish(steering_report_msg);

  // Publish Velocity Report
  auto velocity_report_msg = VelocityReportMsg();
  velocity_report_msg.header.stamp = this->now();
  velocity_report_msg.header.frame_id = base_frame_id_;
  velocity_report_msg.longitudinal_velocity = linear_velocity;
  velocity_report_msg.heading_rate = angular_velocity;
  vehicle_velocity_pub_->publish(velocity_report_msg);
}

/**
 * @brief Callback function for control commands from Autoware
 * @param [in] control_msg Shared pointer to control message containing velocity and steering angle
 * @details Processes control commands to:
 *   - Convert steering angle to angular velocity using bicycle model
 *   - Publish twist command with linear and angular velocities
 */
void NovaCarterInterface::control_cmd_callback(const ControlMsg::ConstSharedPtr control_msg)
{
  RCLCPP_DEBUG(this->get_logger(), "Received control command: velocity = %f, steering_angle = %f",
              control_msg->longitudinal.velocity, control_msg->lateral.steering_tire_angle);

  // Clamp linear velocity within bounds
  const double linear_velocity = std::clamp(
    static_cast<double>(control_msg->longitudinal.velocity),
    -maximum_linear_velocity_,
    maximum_linear_velocity_);

  // Calculate and clamp angular velocity
  const double raw_angular_velocity = 
    linear_velocity / virtual_wheel_base_ * std::tan(control_msg->lateral.steering_tire_angle);
  const double angular_velocity = std::clamp(
    raw_angular_velocity,
    -maximum_angular_velocity_,
    maximum_angular_velocity_);

  // Publish twist command    
  TwistMsg twist_msg = TwistMsg();
  twist_msg.linear.x = linear_velocity;
  twist_msg.angular.z = angular_velocity;
  twist_pub_->publish(twist_msg);
}

}  // namespace autoware::nova_carter_interface

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::nova_carter_interface::NovaCarterInterface)