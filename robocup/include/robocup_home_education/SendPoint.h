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

#ifndef ROBOCUP_HOME_EDUCATION_SENDO_POINT_H
#define ROBOCUP_HOME_EDUCATION_SENDO_POINT_H

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#include "robocup_home_education/str_followobj.h"

#include "ros/ros.h"

namespace robocup_home_education
{

class Transmitter : public BT::ActionNodeBase
{
public:
  explicit Transmitter(const std::string& name, const BT::NodeConfiguration& config);

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

} // namespace robocup_home_education

#endif  // ROBOCUP_HOME_EDUCATION_SENDO_POINT_H