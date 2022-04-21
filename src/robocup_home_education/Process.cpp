
// Copyright 2022 Intelligent Robotics Lab
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

#include "robocup_home_education/Process.h"
#include "ros/ros.h"
#include "behaviortree_cpp_v3/behavior_tree.h"


namespace robocup_home_education
{

Process::Process(const std::string& name, const BT::NodeConfiguration& config)
:BT::ActionNodeBase(name, config), counter_(0)
{
}

void
Process::halt()
{
  ROS_INFO("move halt");
}

BT::NodeStatus
Process::tick()
{
  person = getInput<objectinimage>("person").value();
  std::cerr << "Informacion de la persona: "<< person.depth << " Profundidad; "<< person.detected << " Deteccion" << std::endl;
  //MUERE AQUI
  BT::TreeNode::setOutput("person", person);
  std::cerr << "Muero" << std::endl;  //Llega hasta aqui 
  return BT::NodeStatus::RUNNING;
}

}  // namespace robocup_home_education


#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<robocup_home_education::Process>("Process");
}
