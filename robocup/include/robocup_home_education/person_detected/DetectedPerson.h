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

#ifndef ROBOCUP_HOME_DETECTEDPERSON_H
#define ROBOCUP_HOME_DETECTEDPERSON_H

#include <string>

#include "tf2/transform_datatypes.h"
#include "tf2_ros/transform_listener.h"
#include "tf2/LinearMath/Transform.h"
#include "geometry_msgs/TransformStamped.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2/convert.h"

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#include "robocup_home_education/str_followobj.h"
#include "robocup_home_education/person_detected/detect_BBX.h"
#include "robocup_home_education/person_detected/person_tf.h"
#include "msgs/pos_person.h"

#include "ros/ros.h"

namespace robocup_home_education
{

class Detected : public BT::ActionNodeBase
{
public:
  explicit Detected(const std::string& name, const BT::NodeConfiguration& config);

  void cb_person(const msgs::pos_person::ConstPtr& person);

  void halt();

  BT::NodeStatus tick();

  static BT::PortsList providedPorts()
  {
    return { BT::OutputPort<PointTF>("point")};
  }

private:
  ros::NodeHandle nh_;

  ros::Subscriber sub_pos;

  PersonBBX personbbx;
  PersonTf persontf;

  tf2_ros::Buffer buffer;
  tf2_ros::TransformListener listener;

  geometry_msgs::TransformStamped bf2person_msg;
  tf2::Stamped<tf2::Transform> bf2person;

  PointTF p_tf;

  bool detected_;
};

} // namespace robocup_home_education

#endif  // ROBOCUP_HOME_DETECTEDPERSON_H