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
#include "jitsuyo/cli.hpp"

TEST(CliTest, Uppercased)
{
  std::string s = "hello";
  EXPECT_EQ(jitsuyo::uppercased(s), std::string("HELLO")) << "`hello` must be uppercased";
}

TEST(CliTest, Lowercased)
{
  std::string s = "HELLO";
  EXPECT_EQ(jitsuyo::lowercased(s), std::string("hello")) << "`HELLO` must be lowercased";
}

TEST(CliTest, FgWhite)
{
  std::string s = "hello";
  EXPECT_EQ(jitsuyo::fg_white() + s, std::string("\033[37mhello")) << "`hello` must be white";
}

TEST(CliTest, FgBlack)
{
  std::string s = "hello";
  EXPECT_EQ(jitsuyo::fg_black() + s, std::string("\033[30mhello")) << "`hello` must be black";
}

TEST(CliTest, FgRed)
{
  std::string s = "hello";
  EXPECT_EQ(jitsuyo::fg_red() + s, std::string("\033[31mhello")) << "`hello` must be red";
}

TEST(CliTest, FgGreen)
{
  std::string s = "hello";
  EXPECT_EQ(jitsuyo::fg_green() + s, std::string("\033[32mhello")) << "`hello` must be green";
}

TEST(CliTest, FgYellow)
{
  std::string s = "hello";
  EXPECT_EQ(jitsuyo::fg_yellow() + s, std::string("\033[33mhello")) << "`hello` must be yellow";
}

TEST(CliTest, FgBlue)
{
  std::string s = "hello";
  EXPECT_EQ(jitsuyo::fg_blue() + s, std::string("\033[34mhello")) << "`hello` must be blue";
}

TEST(CliTest, FgMagenta)
{
  std::string s = "hello";
  EXPECT_EQ(jitsuyo::fg_magenta() + s, std::string("\033[35mhello")) << "`hello` must be magenta";
}

TEST(CliTest, FgCyan)
{
  std::string s = "hello";
  EXPECT_EQ(jitsuyo::fg_cyan() + s, std::string("\033[36mhello")) << "`hello` must be cyan";
}

TEST(CliTest, Reset)
{
  std::string s = "hello";
  EXPECT_EQ(jitsuyo::reset() + s, std::string("\033[0mhello")) << "`hello` must be reset";
}

TEST(CliTest, PrintLine)
{
  testing::internal::CaptureStdout();
  jitsuyo::print_line();
  std::string output = testing::internal::GetCapturedStdout();
  EXPECT_EQ(
    output,
    std::string("\033[37m================================================================\n"))
    << "Line must be printed";
}

TEST(CliTest, Print)
{
  testing::internal::CaptureStdout();
  jitsuyo::print(std::string("hello"));
  std::string output = testing::internal::GetCapturedStdout();
  EXPECT_EQ(output, std::string("hello\n")) << "`hello` must be printed";
}

TEST(CliTest, PrintError)
{
  testing::internal::CaptureStdout();
  jitsuyo::print_error(std::string("hello"));
  std::string output = testing::internal::GetCapturedStdout();
  EXPECT_EQ(output, std::string("\033[31mhello\n")) << "`hello` must be printed as an error";
}

TEST(CliTest, PrintBlock)
{
  testing::internal::CaptureStdout();
  jitsuyo::print_block(std::string("hello"));
  std::string output = testing::internal::GetCapturedStdout();
  EXPECT_EQ(
    output,
    std::string("\033[37m[ \033[34mHELLO\033[37m ]\n")) << "`hello` must be printed as a block";
}

TEST(CliTest, PrintParameter)
{
  testing::internal::CaptureStdout();
  jitsuyo::print_parameter(std::string("key"), std::string("value"));

  std::string output = testing::internal::GetCapturedStdout();
  EXPECT_EQ(
    output,
    std::string("key: \033[33mvalue\033[0m\n")) << "`key: value` must be printed as a parameter";
}

TEST(CliTest, PrintParameterBool)
{
  testing::internal::CaptureStdout();
  jitsuyo::print_parameter(std::string("key"), true);

  std::string output = testing::internal::GetCapturedStdout();
  EXPECT_EQ(
    output,
    std::string("key: \033[33mtrue\033[0m\n")) << "`key: true` must be printed as a parameter";
}

TEST(CliTest, PrintParameterInt)
{
  testing::internal::CaptureStdout();
  jitsuyo::print_parameter(std::string("key"), 42);

  std::string output = testing::internal::GetCapturedStdout();
  EXPECT_EQ(
    output,
    std::string("key: \033[33m42\033[0m\n")) << "`key: 42` must be printed as a parameter";
}

TEST(CliTest, PrintParameterFloat)
{
  testing::internal::CaptureStdout();
  jitsuyo::print_parameter(std::string("key"), 42.0f);

  std::string output = testing::internal::GetCapturedStdout();
  EXPECT_EQ(
    output,
    std::string("key: \033[33m42.00\033[0m\n")) << "`key: 42.00` must be printed as a parameter";
}

TEST(CliTest, PrintParameterBlock)
{
  testing::internal::CaptureStdout();
  jitsuyo::print_parameter_block(std::string("key"), std::string("value"));

  std::string output = testing::internal::GetCapturedStdout();
  EXPECT_EQ(
    output,
    std::string("\033[37m[ \033[34mKEY\033[37m \033[33mvalue\033[37m ] \033[0m")) <<
    "`key value` must be printed as a parameter block";
}

TEST(CliTest, PrintParameterBlockBool)
{
  testing::internal::CaptureStdout();
  jitsuyo::print_parameter_block(std::string("key"), true);

  std::string output = testing::internal::GetCapturedStdout();
  EXPECT_EQ(
    output,
    std::string("\033[37m[ \033[34mKEY\033[37m \033[33mtrue\033[37m ] \033[0m")) <<
    "`key true` must be printed as a parameter block";
}

TEST(CliTest, PrintParameterBlockInt)
{
  testing::internal::CaptureStdout();
  jitsuyo::print_parameter_block(std::string("key"), 42);

  std::string output = testing::internal::GetCapturedStdout();
  EXPECT_EQ(
    output,
    std::string("\033[37m[ \033[34mKEY\033[37m \033[33m42\033[37m ] \033[0m"))
    << "`key 42` must be printed as a parameter block";
}

TEST(CliTest, PrintParameterBlockFloat)
{
  testing::internal::CaptureStdout();
  jitsuyo::print_parameter_block(std::string("key"), 42.0f);

  std::string output = testing::internal::GetCapturedStdout();
  EXPECT_EQ(
    output,
    std::string("\033[37m[ \033[34mKEY\033[37m \033[33m42.00\033[37m ] \033[0m")) <<
    "`key 42.00` must be printed as a parameter block";
}

TEST(CliTest, Clear)
{
  testing::internal::CaptureStdout();
  jitsuyo::clear();
  std::string output = testing::internal::GetCapturedStdout();
  EXPECT_EQ(output, std::string("\033[2J\033[2H\n")) << "Screen must be cleared";
}
