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

#include "robocup_home_education/person_detected/DetectedPerson.h"
#include "robocup_home_education/str_followobj.h"

#include "behaviortree_cpp_v3/behavior_tree.h"

#include "ros/ros.h"


namespace robocup_home_education
{
  Detected::Detected(const std::string& name, const BT::NodeConfiguration& config)
  : BT::ActionNodeBase(name, config),
    listener(buffer)
  {
    //personbbx.callback_bbx();
    sub_pos = nh_.subscribe("/robocup_home/Position_human", 1, &Detected::cb_person, this);
  }

  void
  Detected::cb_person(const msgs::pos_person::ConstPtr& person)
  {
    ROS_INFO("ENTRO AL PERSONCB");
    detected_ = person->detected_;
  }

  void
  Detected::halt()
  {
    ROS_INFO("Detected halt");
  }

  BT::NodeStatus
  Detected::tick()
  {
    geometry_msgs::TransformStamped bf2person_msg;
    tf2::Stamped<tf2::Transform> bf2person;

    std::string error;
    if(!detected_) {
      return BT::NodeStatus::FAILURE;
    }
    if (buffer.canTransform("map", "person/0", ros::Time(0), ros::Duration(0.1), &error))
    {
      bf2person_msg = buffer.lookupTransform("map", "person/0", ros::Time(0));

      tf2::fromMsg(bf2person_msg, bf2person);

      double dist_ = bf2person.getOrigin().length();

      p_tf.x = bf2person.getOrigin().x();
      p_tf.y = bf2person.getOrigin().y();
      p_tf.z = bf2person.getOrigin().z();
      ROS_INFO("******************ENVIO TRANSFORMADA A MOVE******************");
      /*ROS_INFO("map -> person [%lf, %lf, %lf] dist = %lf\t%lf ago",
        p_tf.x,
        p_tf.y,
        p_tf.z,
        dist_,
        (ros::Time::now() - bf2person.stamp_).toSec());*/
      BT::TreeNode::setOutput("point", p_tf);
      return BT::NodeStatus::SUCCESS;
    }
    else
    {
      ROS_ERROR("%s", error.c_str());
      return BT::NodeStatus::FAILURE;
    }
    
  }
} //namespace robocup_home_education


#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<robocup_home_education::Detected>("DetectedPerson");
}
