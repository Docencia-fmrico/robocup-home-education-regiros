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

#include "speech/OfferSeat.h"
#include <string>

namespace speech
{
  OfferSeat::OfferSeat(const std::string& name, const BT::NodeConfiguration& config)
  : BT::ActionNodeBase(name, config),
    nh_("~")
  {}

  void
  OfferSeat::halt()
  {
    ROS_INFO("OfferSeat halt");
  }

  BT::NodeStatus
  OfferSeat::tick()
  {
    BT::TreeNode::getInput("Info", info_);

    if (info_.old)
    {
      forwarder.speak("Please sit in the sofa");
    }
    else
    {
      forwarder.speak("You can seat in this chair if you want");
    }

    return BT::NodeStatus::SUCCESS;
  }

};  // namespace speech

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<speech::OfferSeat>("OfferSeat");
}
