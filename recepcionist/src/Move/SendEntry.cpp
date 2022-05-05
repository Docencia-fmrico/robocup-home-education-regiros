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

#include "recepcionist/SendEntry.h"
#include "recepcionist/str_followobj.h"

#include "behaviortree_cpp_v3/behavior_tree.h"

#include "ros/ros.h"

namespace recepcionist
{
  Entry::Entry(const std::string& name, const BT::NodeConfiguration& config)
  : BT::ActionNodeBase(name, config)
  {
  }
  
  void
  Entry::halt()
  {
    ROS_INFO("Transmitter halt");
  }

  BT::NodeStatus
  Entry::tick()
  {
    ROS_INFO("ENVIO PUNTOS DE ENTRADA");
    p_tf.x = 6.243;
    p_tf.y = 0.005;
    p_tf.z = 0.0;
    p_tf.or_x = 0.0;
    p_tf.or_y = 0.0;
    p_tf.or_z = 0.0;
    p_tf.or_w = 1.0;
    
    BT::TreeNode::setOutput("point", p_tf);
      
    return BT::NodeStatus::SUCCESS;
  }
} //namespace recepcionist


#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<recepcionist::Entry>("SendEntry");
}