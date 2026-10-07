// Copyright (c) 2021 Ichiro ITS
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#ifndef ARUKU__WALKING__NODE__WALKING_NODE_HPP_
#define ARUKU__WALKING__NODE__WALKING_NODE_HPP_

#include <memory>

#include "aruku/walking/node/walking_manager.hpp"
#include "aruku_interfaces/msg/imu.hpp"
#include "aruku_interfaces/msg/set_joints.hpp"
#include "aruku_interfaces/msg/set_walking.hpp"
#include "aruku_interfaces/msg/status.hpp"
#include "rclcpp/rclcpp.hpp"

namespace aruku
{

// Jembatan ROS 2 untuk walking engine.
//
//   sub  /walking/set_walking  (SetWalking) : perintah jalan / berhenti
//   sub  /walking/imu          (Imu)        : orientasi badan untuk balance PID
//   pub  /joint/set_joints     (SetJoints)  : target sudut servo (derajat)
//   pub  /walking/status       (Status)     : status walking + odometri
class WalkingNode
{
public:
  using Imu = aruku_interfaces::msg::Imu;
  using SetJoints = aruku_interfaces::msg::SetJoints;
  using SetWalking = aruku_interfaces::msg::SetWalking;
  using WalkingStatus = aruku_interfaces::msg::Status;

  WalkingNode(rclcpp::Node::SharedPtr node, std::shared_ptr<WalkingManager> walking_manager);

private:
  void update();
  void publish_joints();
  void publish_status();

  rclcpp::Node::SharedPtr node;
  std::shared_ptr<WalkingManager> walking_manager;

  rclcpp::Subscription<SetWalking>::SharedPtr set_walking_subscriber;
  rclcpp::Subscription<Imu>::SharedPtr imu_subscriber;
  rclcpp::Publisher<SetJoints>::SharedPtr set_joints_publisher;
  rclcpp::Publisher<WalkingStatus>::SharedPtr status_publisher;

  rclcpp::TimerBase::SharedPtr timer;
  rclcpp::Time last_time;
};

}  // namespace aruku

#endif  // ARUKU__WALKING__NODE__WALKING_NODE_HPP_
