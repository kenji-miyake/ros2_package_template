// SPDX-License-Identifier: Apache-2.0
// Copyright 2021 Kenji Miyake
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

#ifndef PACKAGE_NAME__PACKAGE_NAME_NODE_HPP__
#define PACKAGE_NAME__PACKAGE_NAME_NODE_HPP__

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "example_interfaces/msg/int32.hpp"
#include "package_name/package_name.hpp"
#include "rclcpp/rclcpp.hpp"

namespace package_name
{
using example_interfaces::msg::Int32;

class PackageNameNode : public rclcpp::Node
{
public:
  explicit PackageNameNode(const rclcpp::NodeOptions & node_options);

  struct NodeParam
  {
    double update_rate_hz{};
  };

private:
  // Subscriber
  rclcpp::Subscription<Int32>::SharedPtr sub_data_{};

  // Callback
  void onData(const Int32::ConstSharedPtr msg);

  // Data Buffer
  Int32::ConstSharedPtr data_{};

  // Publisher
  rclcpp::Publisher<Int32>::SharedPtr pub_data_{};

  // Timer
  rclcpp::TimerBase::SharedPtr timer_{};

  bool isDataReady();
  void onTimer();

  // Parameter Server
  OnSetParametersCallbackHandle::SharedPtr set_param_res_;
  rcl_interfaces::msg::SetParametersResult onSetParam(
    const std::vector<rclcpp::Parameter> & params);

  // Parameter
  NodeParam node_param_{};

  // Core
  PackageName::Input input_{};
  PackageName::Output output_{};
  PackageName::Param core_param_{};
  std::unique_ptr<PackageName> package_name_{};
};

}  // namespace package_name

#endif  // PACKAGE_NAME__PACKAGE_NAME_NODE_HPP__
