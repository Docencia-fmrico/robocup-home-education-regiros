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

#include "recepcionist/SayInfo.h"
#include <string>

namespace recepcionist
{
  SayInfo::SayInfo(const std::string& name, const BT::NodeConfiguration& config)
  : BT::ActionNodeBase(name, config),
    nh_("~")
  {}

  void
  SayInfo::halt()
  {
    ROS_INFO("SayInfo halt");
  }

  BT::NodeStatus
  SayInfo::tick()
  {
    std::ostringstream oss;
    std::string phrase;

    BT::TreeNode::getInput("Info", info_);

    oss << "Their name is " << info_.name << ", and their favorite drink is " << info_.drink;
    phrase = oss.str();

    forwarder.speak(phrase);

    return BT::NodeStatus::SUCCESS;
  }

};  // namespace recepcionist

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<recepcionist::SayInfo>("SayInfo");
}
