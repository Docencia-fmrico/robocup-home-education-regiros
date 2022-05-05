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

#ifndef RECEPCIONIST_SENDSIT_H
#define RECEPCIONIST_SENDSIT_H

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#include "recepcionist/str_followobj.h"

#include "ros/ros.h"

namespace recepcionist
{

class Sit : public BT::ActionNodeBase
{
public:
  explicit Sit(const std::string& name, const BT::NodeConfiguration& config);

  void halt();

  BT::NodeStatus tick();

  static BT::PortsList providedPorts()
  {
    return { BT::OutputPort<PointTF>("point")};
  }

private:
  ros::NodeHandle nh_;

  /*int ind_pos;

  float posiciones[6][3] = {{2.243,6.359,0.0}, {0.427,6.354,0.0}, {0.188,5.898,0.0}, {0.759,3.458,0.0}, {-0.128,4.162,0.0}, {2.295,3.182,0.0}};
  float orientaciones[6][4] = {{0.0,0.0,0.701, 0.713}, {0.0,0.0,0.861,0.509}, {0.0,0.0,0.999, 0.053}, {0.0,0.0,-0.944, 0.330}, {0.0,0.0,0.0,1.0}};
*/
  PointTF p_tf;
};

} // namespace recepcionist

#endif  // RECEPCIONIST_SENDSIT_H