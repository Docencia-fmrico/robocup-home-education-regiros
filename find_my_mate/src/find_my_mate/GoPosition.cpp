// Copyright 2022 Regiros
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

#include "ros/ros.h"
#include "find_my_mate/GoPosition.h"
#include "behaviortree_cpp_v3/behavior_tree.h"
#include <string>

namespace find_my_mate
{
  GoPosition::GoPosition(const std::string& name)
  : BT::ActionNodeBase(name, {}),
    nh_("~"),
    count_(0)
  {}

  void
  GoPosition::halt()
  {
    ROS_INFO("Start halt");
  }

  BT::NodeStatus
  GoPosition::tick()
  {
    while (count_ < 10)
    {
      ROS_INFO("Moving... %d", count_);
      count_++;
      return BT::NodeStatus::RUNNING;
    }

    count_ = 0;
    return BT::NodeStatus::SUCCESS;
  }

};  // namespace find_my_mate

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<find_my_mate::GoPosition>("GoPosition");
}