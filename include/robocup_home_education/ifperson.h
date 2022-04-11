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

#ifndef ROBOCUP_HOME_EDUCATION_IFPERSON_H
#define ROBOCUP_HOME_EDUCATION_IFPERSON_H

#include "string"

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <cv_bridge/cv_bridge.h>

#include <sensor_msgs/Image.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#include "robocup_home_education/str_followobj.h"
#include "robocup_home_education/PIDController.h"

#include "ros/ros.h"

namespace robocup_home_education
{
class ifperson : public BT::ActionNodeBase
{
  public:
    explicit ifperson(const std::string& name, const BT::NodeConfiguration& config);

    void halt();

    BT::NodeStatus tick();

    void callback_bbx(const sensor_msgs::ImageConstPtr& depth, const darknet_ros_msgs::BoundingBoxesConstPtr& boxes);

    static BT::PortsList providedPorts()
    {
      return { BT::OutputPort<struct objectinimage>("person")};
    }

  private:
    ros::NodeHandle nh_;

    message_filters::Subscriber<sensor_msgs::Image> depth_sub_;
    message_filters::Subscriber<darknet_ros_msgs::BoundingBoxes> bbx_sub_;

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, darknet_ros_msgs::BoundingBoxes> MySyncPolicy_bbx;
    message_filters::Synchronizer<MySyncPolicy_bbx> sync_bbx_;

    struct objectinimage person;
    const double ideal_depth_ = 1.0;
    const double ideal_x_ = 320;
};
}  // namespace robocup_home_education

#endif  // ROBOCUP_HOME_EDUCATION_IFPERSON_H
