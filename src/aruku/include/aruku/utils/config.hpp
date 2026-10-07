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

// Helper pembaca JSON (diambil dari paket jitsuyo).

#ifndef ARUKU__UTILS__CONFIG_HPP_
#define ARUKU__UTILS__CONFIG_HPP_

#include <iostream>
#include <string>
#include <type_traits>

#include "keisan/angle/angle.hpp"
#include "nlohmann/json.hpp"

namespace aruku
{

template<typename T>
bool assign_val(const nlohmann::json & json, const std::string & key, T & val)
{
  auto it = json.find(key);
  if (it != json.end()) {
    if constexpr (std::is_same_v<std::remove_reference_t<T>, keisan::Angle<double>>) {
      val = keisan::make_degree(it->get<double>());
    } else {
      it->get_to(val);
    }
    return true;
  }
  std::cout << "Failed to find key `" << key << "`" << std::endl;
  return false;
}

}  // namespace aruku

#endif  // ARUKU__UTILS__CONFIG_HPP_
