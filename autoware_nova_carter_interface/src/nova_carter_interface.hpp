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

#ifndef NOVA_CARTER_INTERFACE_HPP_
#define NOVA_CARTER_INTERFACE_HPP_

#include <rclcpp/rclcpp.hpp>

// Autoware messages
#include <autoware_control_msgs/msg/control.hpp>
#include <autoware_vehicle_msgs/msg/steering_report.hpp>
#include <autoware_vehicle_msgs/msg/velocity_report.hpp>

// ROS messages for Nova Carter
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <string>

namespace autoware::nova_carter_interface
{

class NovaCarterInterface : public rclcpp::Node
{
public:
  NovaCarterInterface(const rclcpp::NodeOptions & options);

private:
  // Type aliases for readability
  using ControlMsg = autoware_control_msgs::msg::Control;
  using OdometryMsg = nav_msgs::msg::Odometry;
  using SteeringReportMsg = autoware_vehicle_msgs::msg::SteeringReport;
  using TwistStampedMsg = geometry_msgs::msg::TwistStamped;
  using VelocityReportMsg = autoware_vehicle_msgs::msg::VelocityReport;


  // Subscribers From Autoware
  rclcpp::Subscription<ControlMsg>::SharedPtr control_cmd_sub_;

  // Subscribers From Nova Carter
  rclcpp::Subscription<OdometryMsg>::SharedPtr odom_sub_;

  // Publishers To Nova Carter
  rclcpp::Publisher<TwistStampedMsg>::SharedPtr twist_pub_;

  // Publishers To Autoware
  rclcpp::Publisher<VelocityReportMsg>::SharedPtr vehicle_twist_pub_;
  rclcpp::Publisher<SteeringReportMsg>::SharedPtr steering_status_pub_;

  // ROS Parameters
  std::string base_frame_id_;
  double maximum_angular_velocity_;     // [rad/s]
  double maximum_linear_velocity_;      // [m/s]
  double virtual_wheel_base_;           // [m]

  // Input Values
  ControlMsg::ConstSharedPtr control_cmd_ptr_;
  OdometryMsg::ConstSharedPtr odom_ptr_;

  // Callbacks
  void control_cmd_callback(const ControlMsg::ConstSharedPtr msg);
  void odometry_callback(const OdometryMsg::ConstSharedPtr msg);

};

}  // namespace autoware::nova_carter_interface

#endif  // NOVA_CARTER_INTERFACE_HPP_
