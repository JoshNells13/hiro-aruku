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

#ifndef JITSUYO__CLI_HPP_
#define JITSUYO__CLI_HPP_

#include <string>

namespace jitsuyo
{

std::string uppercased(std::string str);
std::string lowercased(std::string str);

void clear();
void end_line();

void print_line();

void print(const std::string & value);
void print_error(const std::string & value);
void print_block(const std::string & value);

void print_parameter(const std::string & key, const std::string & value);
void print_parameter(const std::string & key, bool value);
void print_parameter(const std::string & key, int value);
void print_parameter(const std::string & key, float value);
void print_parameter(const std::string & key, double value);

void print_parameter_block(const std::string & key, const std::string & value);
void print_parameter_block(const std::string & key, bool value);
void print_parameter_block(const std::string & key, int value);
void print_parameter_block(const std::string & key, float value);
void print_parameter_block(const std::string & key, double value);

std::string reset();

std::string fg_white();
std::string fg_black();
std::string fg_red();
std::string fg_green();
std::string fg_yellow();
std::string fg_blue();
std::string fg_magenta();
std::string fg_cyan();

}  // namespace jitsuyo

#endif  // JITSUYO__CLI_HPP_
