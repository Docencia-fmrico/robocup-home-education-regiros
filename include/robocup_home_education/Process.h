
// Copyright 2019 Intelligent Robotics Lab
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
#ifndef ROBOCUP_HOME_EDUCATION_PROCESS_H
#define ROBOCUP_HOME_EDUCATION_PROCESS_H


#include "robocup_home_education/str_followobj.h"

#include "string"
#include "ros/ros.h"
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

namespace robocup_home_education
{

class Process
{
  public:
    Process(const std::string& name, const BT::NodeConfiguration& config);

    void halt();
    BT::NodeStatus tick();
    
    static BT::PortsList providedPorts1()
    {
      return { BT::InputPort<struct objectinimage>("person") };
    }

    static BT::PortsList providedPorts2()
    {
      return { BT::OutputPort<struct objectinimage>("person")};
    }

  private:
    //ros::NodeHandle nh_;
    int counter_;
    struct objectinimage person;
};

}  // namespace robocup_home_education

#endif  // ROBOCUP_HOME_EDUCATION_PROCESS_H
