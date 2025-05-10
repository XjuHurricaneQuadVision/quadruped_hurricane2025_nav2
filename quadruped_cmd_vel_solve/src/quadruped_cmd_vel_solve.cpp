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

#include "quadruped_cmd_vel_solve/quadruped_cmd_vel_solve.hpp"

using std::placeholders::_1;

namespace quadruped_cmd_vel_solve
{

    QuadrupedCmdVelSolve::QuadrupedCmdVelSolve()
        : Node("quadruped_cmd_vel_solve")
    {

        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel",
            20,
            std::bind(&QuadrupedCmdVelSolve::cmdVelCallback, this, _1)
        );

        serial_cmd_pub_ = this->create_publisher<std_msgs::msg::String>(
            "/serial_cmd", 
            20
        );

        RCLCPP_INFO(this->get_logger(), "quadruped_cmd_vel_solve node started.");
    }

    void QuadrupedCmdVelSolve::cmdVelCallback(
        const geometry_msgs::msg::Twist::SharedPtr msg_)
    {
        auto action = 1;
        
        float linear_x = msg_->linear.x;
        float linear_y = msg_->linear.y;

        float angular_z = msg_->angular.z;

        RCLCPP_INFO(this->get_logger(), "linear_x: %f, linear_y: %f, angular_z: %f", linear_x, linear_y, angular_z);


        if (linear_x >= 0.5){
            if (linear_y <= 0.5 && linear_y >= -0.5){
                RCLCPP_INFO(this->get_logger(), "直走");
                action = 12;
            }
            else if (linear_y > 0.5){
                RCLCPP_INFO(this->get_logger(), "边走边左转");
                action = 13;
            }
            else if (linear_y < -0.5){
                RCLCPP_INFO(this->get_logger(), "边走边右转");
                action = 5;
            }
        }
        else if (linear_x < 0.5){
            if (linear_y <= 0.5 && linear_y >= -0.5){
                RCLCPP_INFO(this->get_logger(), "站立");
                action = 16;
            }
            else if (linear_y > 0.5){
                RCLCPP_INFO(this->get_logger(), "左转");
                action = 15;
            }
            else if (linear_y < -0.5){
                RCLCPP_INFO(this->get_logger(), "右转");
                action = 8;
            }
        }


        std::string action_str = std::to_string(action);
        
        std_msgs::msg::String serial_cmd;
        serial_cmd.data = action_str;
        serial_cmd_pub_->publish(serial_cmd);
      

    }

} // namespace quadruped_cmd_vel_solve

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto quadruped_cmd_vel_solve = std::make_shared<quadruped_cmd_vel_solve::QuadrupedCmdVelSolve>();
    rclcpp::spin(quadruped_cmd_vel_solve);
    rclcpp::shutdown();
    return 0;
}
