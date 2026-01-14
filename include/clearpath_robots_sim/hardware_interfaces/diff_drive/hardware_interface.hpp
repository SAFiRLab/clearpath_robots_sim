/**
 *  \file
 *  \brief      Base DiffDrive hardware interface class
 *  \author     Roni Kreinin <rkreinin@clearpathrobotics.com>
 *  \author     Tony Baltovski <tbaltovski@clearpathrobotics.com>
 *  \copyright  Copyright (c) 2023, Clearpath Robotics, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Clearpath Robotics, Inc. nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL CLEARPATH ROBOTICS, INC. BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Please send comments, questions, or patches to code@clearpathrobotics.com
 *
 */

#ifndef CLEARPATH_ROBOTS_SIM__DIFF_DRIVE_HARDWARE_INTERFACE_HPP_
#define CLEARPATH_ROBOTS_SIM__DIFF_DRIVE_HARDWARE_INTERFACE_HPP_

#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"


namespace clearpath_robots_sim_hardware_interfaces
{

class DiffDriveSimHardwareInterface: public rclcpp::Node
{
public:
    explicit DiffDriveSimHardwareInterface(std::string node_name);
    void drive_command(const float & left_wheel, const float & right_wheel, const int8_t & mode);

private:
    // ROS Publishers
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr left_wheel_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr right_wheel_pub_;
};

}  // namespace clearpath_robots_sim_hardware_interfaces

#endif  // CLEARPATH_ROBOTS_SIM__DIFF_DRIVE_HARDWARE_INTERFACE_HPP_