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

#include "find_my_mate/SendPoint.h"
#include "find_my_mate/str_followobj.h"

#include "behaviortree_cpp_v3/behavior_tree.h"

#include "ros/ros.h"

namespace find_my_mate
{
  Transmitter::Transmitter(const std::string& name, const BT::NodeConfiguration& config)
  : BT::ActionNodeBase(name, config)
  {
    ind_pos = 0;
  }
  
  void
  Transmitter::halt()
  {
    ROS_INFO("Transmitter halt");
  }

  BT::NodeStatus
  Transmitter::tick()
  {
    ROS_INFO("ENVIO PUNTOS DEL ARRAY %d", ind_pos);
    p_tf.x = posiciones[ind_pos][0];
    p_tf.y = posiciones[ind_pos][1];
    p_tf.z = posiciones[ind_pos][2];
    p_tf.or_x = orientaciones[ind_pos][0];
    p_tf.or_y = orientaciones[ind_pos][1];
    p_tf.or_z = orientaciones[ind_pos][2];
    p_tf.or_w = orientaciones[ind_pos][3];
    ind_pos++;

    if(ind_pos == 6) {
      ind_pos = 0;
    }
    
    BT::TreeNode::setOutput("point", p_tf);
      
    return BT::NodeStatus::SUCCESS;
  }
} //namespace find_my_mate


#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<find_my_mate::Transmitter>("SendPoint");
}