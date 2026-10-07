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

// Model sendi sederhana (diambil dari paket tachimawari) agar aruku tidak lagi
// bergantung pada paket hardware Dynamixel.

#ifndef ARUKU__WALKING__MODEL__JOINT_HPP_
#define ARUKU__WALKING__MODEL__JOINT_HPP_

#include <array>
#include <cstdint>

#include "keisan/angle/angle.hpp"

namespace aruku
{

class JointId
{
public:
  enum : uint8_t {
    // head
    NECK_YAW = 19,
    NECK_PITCH = 20,

    // left arm
    LEFT_SHOULDER_PITCH = 2,
    LEFT_SHOULDER_ROLL = 4,
    LEFT_ELBOW = 6,

    // right arm
    RIGHT_SHOULDER_PITCH = 1,
    RIGHT_SHOULDER_ROLL = 3,
    RIGHT_ELBOW = 5,

    // left leg
    LEFT_HIP_YAW = 8,
    LEFT_HIP_ROLL = 10,
    LEFT_HIP_PITCH = 12,
    LEFT_KNEE = 14,
    LEFT_ANKLE_PITCH = 16,
    LEFT_ANKLE_ROLL = 18,

    // right leg
    RIGHT_HIP_YAW = 7,
    RIGHT_HIP_ROLL = 9,
    RIGHT_HIP_PITCH = 11,
    RIGHT_KNEE = 13,
    RIGHT_ANKLE_PITCH = 15,
    RIGHT_ANKLE_ROLL = 17,

    // jumlah slot array yang aman untuk diindeks dengan ID
    COUNT = 21,
  };

  // Sendi yang digerakkan walking engine (lengan + kaki, tanpa kepala)
  static constexpr std::array<uint8_t, 18> body_ids = {
    RIGHT_SHOULDER_PITCH, LEFT_SHOULDER_PITCH, RIGHT_SHOULDER_ROLL, LEFT_SHOULDER_ROLL,
    RIGHT_ELBOW, LEFT_ELBOW,
    RIGHT_HIP_YAW, LEFT_HIP_YAW, RIGHT_HIP_ROLL, LEFT_HIP_ROLL,
    RIGHT_HIP_PITCH, LEFT_HIP_PITCH, RIGHT_KNEE, LEFT_KNEE,
    RIGHT_ANKLE_PITCH, LEFT_ANKLE_PITCH, RIGHT_ANKLE_ROLL, LEFT_ANKLE_ROLL,
  };
};

class Joint
{
public:
  enum { CENTER_VALUE = 2048 };

  static int angle_to_value(const keisan::Angle<double> & angle)
  {
    return angle.degree() / 360.0 * 4096.0;
  }

  static keisan::Angle<double> value_to_angle(int value)
  {
    return keisan::make_degree(value / 4096.0 * 360.0);
  }

  explicit Joint(uint8_t id, float position = 0.0)
  : id(id), position(keisan::make_degree(position)) {}

  uint8_t get_id() const { return id; }

  void set_position(float position) { this->position = keisan::make_degree(position); }
  float get_position() const { return position.normalize().degree(); }

  void set_position_value(int value) { position = value_to_angle(value - CENTER_VALUE); }
  int get_position_value() const { return angle_to_value(position) + CENTER_VALUE; }

private:
  uint8_t id;
  keisan::Angle<double> position;
};

}  // namespace aruku

#endif  // ARUKU__WALKING__MODEL__JOINT_HPP_
