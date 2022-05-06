
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

#include <string>

#include "find_my_mate/GoPosition.h"

#include "behaviortree_cpp_v3/behavior_tree.h"

#include "ros/ros.h"

namespace find_my_mate
{

GoPoint::GoPoint(
  const std::string& name,
  const std::string & action_name,
  const BT::NodeConfiguration & config)
: BTNavAction(name, action_name, config), counter_(0),
  listener(buffer)
{
}

void
GoPoint::on_halt()
{
  ROS_INFO("GoPoint halt");
}

void
GoPoint::on_start()
{
  move_base_msgs::MoveBaseGoal goal;
  
  p = getInput<PointTF>("point").value();
  std::cerr << "PUNTOS: [" << p.x << ", " << p.y << ", " << p.z << "]" << std::endl;
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();
  goal.target_pose.pose.position.x = p.x;
  goal.target_pose.pose.position.y = p.y;
  goal.target_pose.pose.position.z = p.z;
  goal.target_pose.pose.orientation.x = p.or_x;
  goal.target_pose.pose.orientation.y = p.or_y;
  goal.target_pose.pose.orientation.z = p.or_z;
  goal.target_pose.pose.orientation.w = p.or_w;

  set_goal(goal);

  ROS_INFO("GoPoint start");
}

BT::NodeStatus
GoPoint::on_tick()
{
  ROS_INFO("GoPoint tick");
  p = getInput<PointTF>("point").value();
  std::cerr << "PUNTOS: [" << p.x << ", " << p.y << ", " << p.z << "]" << std::endl;
  
  ROS_INFO("counter = %d", counter_);
  if (counter_++ == 20)
  {
    std::cerr << "New Goal===========================" << std::endl;
    move_base_msgs::MoveBaseGoal goal;

    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose.position.x = p.x;
    goal.target_pose.pose.position.y = p.y;
    goal.target_pose.pose.position.z = 0.0;
    goal.target_pose.pose.orientation.x = p.or_x;
    goal.target_pose.pose.orientation.y = p.or_y;
    goal.target_pose.pose.orientation.z = p.or_z;
    goal.target_pose.pose.orientation.w = p.or_w;
    counter_ = 0;
    set_goal(goal);
  }

  return BT::NodeStatus::RUNNING;
}

void
GoPoint::on_feedback(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback)
{
	ROS_INFO("Current count %lf", feedback->base_position.pose.position.x);
}

}  // namespace find_my_mate

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
  [](const std::string & name, const BT::NodeConfiguration & config)
  {
    return std::make_unique<find_my_mate::GoPoint>(
      name, "move_base", config);
  };

  factory.registerBuilder<find_my_mate::GoPoint>(
    "GoPoint", builder);
}


/*
  geometry_msgs::TransformStamped bf2base_msg;
  tf2::Stamped<tf2::Transform> bf2base;
  std::string error;

  if (buffer.canTransform("map", "base_link", ros::Time(0), ros::Duration(0.1), &error))
  {
    bf2base_msg = buffer.lookupTransform("map", "base_link", ros::Time(0));

    tf2::fromMsg(bf2base_msg, bf2base);

    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose.position.x = bf2base.getOrigin().x();
    goal.target_pose.pose.position.y = bf2base.getOrigin().y();
    goal.target_pose.pose.position.z = bf2base.getOrigin().z();
    goal.target_pose.pose.orientation.x = bf2base.getRotation().x();
    goal.target_pose.pose.orientation.y = bf2base.getRotation().y();
    goal.target_pose.pose.orientation.z = bf2base.getRotation().z();
    goal.target_pose.pose.orientation.w = bf2base.getRotation().w();

    set_goal(goal);

    ROS_INFO("GoPoint start");
  }*/
