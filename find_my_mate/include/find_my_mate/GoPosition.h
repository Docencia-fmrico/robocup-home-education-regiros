<<<<<<< HEAD

// Copyright 2022 Intelligent Robotics Lab
=======
// Copyright 2022 Regiros
>>>>>>> bc0b060bff163a4fd9002432731bb664a4d17dc0
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

#ifndef FIND_MY_MATE_GOPOSITION_H
#define FIND_MY_MATE_GOPOSITION_H

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#include <move_base_msgs/MoveBaseAction.h>
#include "find_my_mate/BTNavAction.h"
#include "find_my_mate/str_followobj.h"

#include "nav_msgs/GetPlan.h"
#include "geometry_msgs/PointStamped.h"
#include "navfn/MakeNavPlan.h"

#include <actionlib/client/simple_action_client.h>
#include <string>

#include "tf2/transform_datatypes.h"
#include "tf2_ros/transform_listener.h"
#include "tf2/LinearMath/Transform.h"
#include "geometry_msgs/TransformStamped.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2/convert.h"
#include "geometry_msgs/Point.h"
#include "msgs/pos_person.h"

namespace find_my_mate
{

class GoPoint : public BTNavAction
{
  public:
    explicit GoPoint(const std::string& name,
    const std::string & action_name,
    const BT::NodeConfiguration & config);

    void on_halt() override;

    BT::NodeStatus on_tick() override;;
    void on_start() override;
    void on_feedback(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback) override;

    static BT::PortsList providedPorts()
    {
      return { BT::InputPort<PointTF>("point")};
    }

  private:
    ros::NodeHandle n;

    tf2_ros::Buffer buffer;
    tf2_ros::TransformListener listener;
    int counter_;
    PointTF p;
};

}  // namespace find_my_mate

#endif  //FIND_MY_MATE_GOPOSITION_H
