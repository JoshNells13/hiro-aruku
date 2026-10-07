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

#include "aruku/walking/node/walking_node.hpp"

#include <chrono>
#include <memory>

#include "keisan/keisan.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

namespace aruku
{

WalkingNode::WalkingNode(
  rclcpp::Node::SharedPtr node, std::shared_ptr<WalkingManager> walking_manager)
: node(node), walking_manager(walking_manager)
{
  set_walking_subscriber = node->create_subscription<SetWalking>(
    "/walking/set_walking", 10, [this](const SetWalking::SharedPtr message) {
      if (message->run) {
        this->walking_manager->run(
          message->x_move, message->y_move, message->a_move, message->aim_on);
      } else {
        this->walking_manager->stop();
      }
    });

  imu_subscriber = node->create_subscription<Imu>(
    "/walking/imu", 10, [this](const Imu::SharedPtr message) {
      this->walking_manager->update_orientation(keisan::make_degree(message->yaw));
      this->walking_manager->update_imu(
        keisan::make_degree(message->roll), keisan::make_degree(message->pitch));
    });

  set_joints_publisher = node->create_publisher<SetJoints>("/joint/set_joints", 10);
  status_publisher = node->create_publisher<WalkingStatus>("/walking/status", 10);

  last_time = node->now();
  timer = node->create_wall_timer(8ms, [this]() {update();});
}

void WalkingNode::update()
{
  rclcpp::Time now = node->now();
  double dt = (now - last_time).seconds();
  last_time = now;
  if (dt > 0.0 && dt < 0.1) {
    walking_manager->set_delta_time(dt);
  }

  // Kinematika selalu dihitung: saat diam menghasilkan pose berdiri,
  // saat berjalan menghasilkan pola langkah.
  if (walking_manager->process()) {
    publish_joints();
  }
  publish_status();
}

void WalkingNode::publish_joints()
{
  SetJoints message;
  for (const auto & joint : walking_manager->get_joints()) {
    aruku_interfaces::msg::Joint joint_msg;
    joint_msg.id = joint.get_id();
    joint_msg.position = joint.get_position();
    message.joints.push_back(joint_msg);
  }
  set_joints_publisher->publish(message);
}

void WalkingNode::publish_status()
{
  const auto & kinematic = walking_manager->get_kinematic();

  WalkingStatus message;
  message.is_running = walking_manager->is_running();
  message.x_amplitude = kinematic.get_x_move_amplitude();
  message.y_amplitude = kinematic.get_y_move_amplitude();
  message.a_amplitude = kinematic.get_a_move_amplitude();
  message.odometry.x = walking_manager->get_position().x;
  message.odometry.y = walking_manager->get_position().y;
  status_publisher->publish(message);
}

}  // namespace aruku
