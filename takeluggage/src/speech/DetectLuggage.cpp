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

#include "ros/ros.h"
#include "takeluggage/DetectLuggage.h"
#include "behaviortree_cpp_v3/behavior_tree.h"
#include <string>

namespace takeluggage
{
  DetectLuggage::DetectLuggage(const std::string& name)
  : BT::ActionNodeBase(name, {}),
    nh_("~"),
    firsttick_(true)
  {
    pub_turn_ = nh_.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1);
  }

  void
  DetectLuggage::halt()
  {
    ROS_INFO("Start halt");
  }

  BT::NodeStatus
  DetectLuggage::tick()
  {
    ROS_INFO("First tick: %d", firsttick_);

    if (firsttick_)
    {
      firsttick_ = false;
      forwarder.done_ = false;
      forwarder.speak("Which is your luggage?");
      forwarder.listen();
    }

    ROS_INFO("Done Start: %d", forwarder.done_);
    if (forwarder.done_)
    { 
      forwarder.speak(forwarder.response_);
      geometry_msgs::Twist cmd;
      ros::Time time_now;
      cmd.linear.x = 0;

      if (forwarder.param_ == "right")
      {
        cmd.angular.z = -0.33;
      }
      else
      {
        cmd.angular.z = 0.3;
      }

      pub_turn_.publish(cmd);
      time_now = ros::Time::now();
      while ((ros::Time::now() - press_ts_).toSec() > 1.0 ) {}
      return BT::NodeStatus::SUCCESS;
    }
    else
    {
      return BT::NodeStatus::RUNNING;
    }
  }

};  // namespace recepcionist

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<takeluggage::DetectLuggage>("DetectLuggage");
}
