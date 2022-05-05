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

#include "speech/AskName.h"
#include <string>

namespace speech
{
  AskName::AskName(const std::string& name, const BT::NodeConfiguration& config)
  : BT::ActionNodeBase(name, config),
    nh_("~"),
    firsttick_(true)
  {}

  void
  AskName::halt()
  {
    ROS_INFO("AskName halt");
  }

  BT::NodeStatus
  AskName::tick()
  {
    ROS_INFO("First tick Name: %d", firsttick_);
    if (firsttick_)
    {
      firsttick_ = false;
      forwarder.done_ = false;
      forwarder.speak("What is your name?");
    }

    ROS_INFO("Done Name: %d", forwarder.done_);
    if (forwarder.done_)
    {
      forwarder.speak(forwarder.response_);
      forwarder.done_ = false;
      firsttick_ = true;
      ROS_INFO("PARAMETRO: %s", forwarder.param_.c_str());
      info_.name = forwarder.param_;
      BT::TreeNode::setOutput("Info", info_);
      return BT::NodeStatus::SUCCESS;
    }
    else
    {
      forwarder.listen();
      return BT::NodeStatus::RUNNING;
    }
  }

};  // namespace speech

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<speech::AskName>("AskName");
}
