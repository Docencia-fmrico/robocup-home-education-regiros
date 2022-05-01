
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

#ifndef ROBOCUP_HOME_EDUCATION_MOVE_H
#define ROBOCUP_HOME_EDUCATION_MOVE_H

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#include <move_base_msgs/MoveBaseAction.h>
#include "robocup_home_education/BTNavAction.h"
#include "robocup_home_education/str_followobj.h"


#include <string>

namespace robocup_home_education
{

class Move : public BTNavAction
{
  public:
    explicit Move(const std::string& name,
    const std::string & action_name,
    const BT::NodeConfiguration & config);

    void on_halt() override;
    BT::NodeStatus on_tick() override;;
    void on_start() override;
    void on_feedback(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback) override;

    static BT::PortsList providedPorts()
    {
      return { BT::InputPort<struct objectinimage>("person")};
    }

  private:
    int counter_;
    struct objectinimage person;
};

}  // namespace robocup_home_education

#endif  //ROBOCUP_HOME_EDUCATION_MOVE_H