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

#include "speech/AskAge.h"
#include <string>

namespace speech
{
  AskAge::AskAge(const std::string& name, const BT::NodeConfiguration& config)
  : BT::ActionNodeBase(name, config),
    nh_("~"),
    firsttick_(true)
  {}

  void
  AskAge::halt()
  {
    ROS_INFO("AskAge halt");
  }

  BT::NodeStatus
  AskAge::tick()
  {
    ROS_INFO("First tick Drink: %d", firsttick_);
    if (firsttick_)
    {
      firsttick_ = false;
      forwarder.done_ = false;
      forwarder.speak("How old are you?");
    }
    

    ROS_INFO("Done Drink: %d", forwarder.done_);
    if (forwarder.done_)
    {
      forwarder.speak(forwarder.response_);
      int age;

      forwarder.done_ = false;
      firsttick_ = true;
      info_.old = false;
      ROS_INFO("PARAMETRO: %s", forwarder.param_.c_str());
      age = stoi(forwarder.param_);

      BT::TreeNode::getInput("Info", info_);
      if (age >= 50)
      {
        info_.old = true;
      }
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
  factory.registerNodeType<speech::AskAge>("AskAge");
}
