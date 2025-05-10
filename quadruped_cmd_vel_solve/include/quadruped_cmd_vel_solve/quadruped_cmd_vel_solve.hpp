// Copyright 2025 Jackson Huang
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

#ifndef QUADRUPED_CMD_VEL_SOLVE__QUADRUPED_CMD_VEL_SOLVE_HPP_
#define QUADRUPED_CMD_VEL_SOLVE__QUADRUPED_CMD_VEL_SOLVE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/string.hpp"

namespace quadruped_cmd_vel_solve
{

class QuadrupedCmdVelSolve : public rclcpp::Node
{
public:
  QuadrupedCmdVelSolve();

private:
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg_);

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr serial_cmd_pub_;
};

}  // namespace quadruped_cmd_vel_solve

#endif  // QUADRUPED_CMD_VEL_SOLVE__QUADRUPED_CMD_VEL_SOLVE_HPP_
