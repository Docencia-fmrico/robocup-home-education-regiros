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

#ifndef FIND_MY_MATE_SENDREFEREE_H
#define FIND_MY_MATE_SENDREFEREE_H

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#include "find_my_mate/str_followobj.h"

#include "ros/ros.h"

namespace find_my_mate
{

class Referee : public BT::ActionNodeBase
{
public:
  explicit Referee(const std::string& name, const BT::NodeConfiguration& config);

  void halt();

  BT::NodeStatus tick();

  static BT::PortsList providedPorts()
  {
    return { BT::OutputPort<PointTF>("point")};
  }

private:
  ros::NodeHandle nh_;

  PointTF p_tf;
};

} // namespace find_my_mate

#endif  // FIND_MY_MATE_SENDREFEREE_H