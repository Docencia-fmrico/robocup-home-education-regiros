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
#include "find_my_mate/Start.h"
#include "behaviortree_cpp_v3/behavior_tree.h"
#include <string>

namespace find_my_mate
{
  Start::Start(const std::string& name)
  : BT::ActionNodeBase(name, {}),
    nh_("~"),
    firsttick_(true)
  {}

  void
  Start::halt()
  {
    ROS_INFO("Start halt");
  }

  BT::NodeStatus
  Start::tick()
  {
    ROS_INFO("First tick: %d", firsttick_);

    if (firsttick_)
    {
      firsttick_ = false;
      forwarder.done_ = false;
      forwarder.speak("Avoid error");
      forwarder.speak("I'm ready, tell me when you want me to start");
      forwarder.listen();
    }

    ROS_INFO("Done Start: %d", forwarder.done_);
    if (forwarder.done_)
    { 
      forwarder.speak(forwarder.response_);
      return BT::NodeStatus::SUCCESS;
    }
    else
    {
      return BT::NodeStatus::RUNNING;
    }
  }

};  // namespace find_my_mate

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<find_my_mate::Start>("Start");
}