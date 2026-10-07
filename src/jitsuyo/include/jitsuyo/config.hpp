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

#ifndef JITSUYO__CONFIG_HPP_
#define JITSUYO__CONFIG_HPP_

#include "keisan/angle/angle.hpp"

#include <fstream>
#include <iostream>
#include <nlohmann/json.hpp>
#include <string>
#include <type_traits>

namespace jitsuyo
{

template<typename T>
bool assign_val(const nlohmann::json & json, const std::string & key, T & val)
{
  auto it = json.find(key);
  if (it != json.end()) {
    if constexpr (std::is_same_v<typename std::remove_reference<decltype(val)>::type, decltype(keisan::Angle<double>())>) {
      double val_double = it->get<double>();
      val = keisan::make_degree(val_double);
    } else {
      it->get_to(val);
    }
    return true;
  }
  std::cout << "Failed to find key `" << key << "`" << std::endl;
  return false;
}

template<typename T>
typename std::enable_if<std::is_same<T, nlohmann::json>::value ||
  std::is_same<T, nlohmann::ordered_json>::value, bool>::type
save_config(
  const std::string & path, const std::string & file_name,
  const T & data)
{
  std::ofstream file(path + file_name, std::ios::out | std::ios::trunc);
  if (!file.is_open()) {
    std::cout << "Failed to open file `" << path + file_name << "`" << std::endl;
    return false;
  }

  file << std::setw(2) << data << std::endl;
  file.close();
  return true;
}

template<typename T>
typename std::enable_if<std::is_same<T, nlohmann::json>::value ||
  std::is_same<T, nlohmann::ordered_json>::value, bool>::type
load_config(
  const std::string & path, const std::string & file_name, T & data)
{
  std::ifstream file(path + file_name);
  if (!file.is_open()) {
    std::cout << "Failed to open file `" << path + file_name << "`" << std::endl;
    return false;
  }

  data = T::parse(file);
  file.close();
  return true;
}
}  // namespace jitsuyo

#endif  // JITSUYO__CONFIG_HPP_
