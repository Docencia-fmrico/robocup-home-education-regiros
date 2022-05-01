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

#include <string>

#include "robocup_home_education/SendPoint.h"
#include "robocup_home_education/str_followobj.h"

#include "behaviortree_cpp_v3/behavior_tree.h"

#include "ros/ros.h"

namespace robocup_home_education
{
  Transmitter::Transmitter(const std::string& name, const BT::NodeConfiguration& config)
  : BT::ActionNodeBase(name, config)
  {
  }

  void
  Transmitter::halt()
  {
    ROS_INFO("Transmitter halt");
  }

  BT::NodeStatus
  Transmitter::tick()
  {
    //3,308, 1,910, 0,000
    //4,255, 2,625
    //Poner punto donde se encuentra el arbitro, servirá para ida y vuelta
    p_tf.x = 4.255;
    p_tf.y = 2.625;
    p_tf.z = 0.000;
    //Position(-3,456, 1,738, 0,000) Inicial
    BT::TreeNode::setOutput("point", p_tf);
      
    return BT::NodeStatus::SUCCESS;
  }
} //namespace robocup_home_education


#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<robocup_home_education::Transmitter>("SendPoint");
}