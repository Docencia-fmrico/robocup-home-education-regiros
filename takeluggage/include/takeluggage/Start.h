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

#ifndef TAKELUGGAGE_START_H
#define TAKELUGGAGE_START_H

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "takeluggage/Chat.h"
#include <string>

namespace takeluggage
{
class Start : public BT::ActionNodeBase
{
  public:
    explicit Start(const std::string& name);

    void halt();

    BT::NodeStatus tick();

  private:
    Chat forwarder;
    ros::NodeHandle nh_;
    bool firsttick_ = true;
};
};  // namespace takeluggage

#endif  // RECEPCIONIST_START_H
