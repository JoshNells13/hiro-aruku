// Copyright (c) 2024 ICHIRO ITS
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

#include <string>

#include "gtest/gtest.h"
#include "jitsuyo/linux.hpp"

TEST(LinuxTest, EnvironmentVariable)
{
  EXPECT_EQ(jitsuyo::get_env(std::string("HOME")), std::string("/home/ichiro"))
    << "Home directory must be /home/ichiro";
  EXPECT_EQ(jitsuyo::get_env(std::string("ROS_DISTRO")), std::string("iron"))
    << "ROS distro must be iron";
}

TEST(LinuxTest, RootUser)
{
  EXPECT_FALSE(jitsuyo::is_root()) << "User must be root";
}

TEST(LinuxTest, DirectoryExist)
{
  EXPECT_TRUE(jitsuyo::is_directory_exist(std::string("/home/ichiro"))) <<
    "Home directory must exist";
  EXPECT_TRUE(jitsuyo::is_directory_exist(std::string("/home/ichiro/ros2-ws")))
    << "ROS 2 workspace must exist";
}

TEST(LinuxTest, CreateDirectory)
{
  EXPECT_TRUE(jitsuyo::create_directory(std::string("/tmp/test")))
    << "Test directory must be created";
}

TEST(LinuxTest, FileExist)
{
  EXPECT_TRUE(
    jitsuyo::is_file_exist(
      std::string("/home/ichiro/ros2-ws/src/jitsuyo/CMakeLists.txt")))
    << "CMake file must exist";
}

TEST(LinuxTest, CreateFile)
{
  EXPECT_TRUE(jitsuyo::create_file(std::string("/tmp/test.txt")))
    << "Test file must be created";
}

TEST(LinuxTest, SplitString)
{
  EXPECT_EQ(jitsuyo::split_string(std::string("Hello-World!")), std::string("Hello-World!"))
    << "String must be the same";
  EXPECT_EQ(
    jitsuyo::split_string(std::string("Hello,World!"), std::string(",")), std::string("World!"))
    << "String must be split";
}
