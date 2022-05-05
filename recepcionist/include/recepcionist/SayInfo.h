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

#ifndef RECEPCIONIST_SAYINFO_H
#define RECEPCIONIST_SAYINFO_H

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "recepcionist/PersonInfo.h"
#include "recepcionist/Chat.h"
#include "string"
#include <sstream>

namespace recepcionist
{
class SayInfo : public BT::ActionNodeBase
{
  public:
    explicit SayInfo(const std::string& name, const BT::NodeConfiguration& config);

    void halt();

    BT::NodeStatus tick();

    static BT::PortsList providedPorts()
    {
      return {BT::InputPort<PInfo>("Info")};
    }

  private:
    Chat forwarder;
    PInfo info_;
    ros::NodeHandle nh_;
};
};  // namespace recepcionist

#endif  // RECEPCIONIST_SAYINFO_H
