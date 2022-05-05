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

#include "find_my_mate/RotateUPerson.h"
#include "find_my_mate/str_info.h"

#include "behaviortree_cpp_v3/behavior_tree.h"

#include "ros/ros.h"

namespace find_my_mate
{
  RotateUPerson::RotateUPerson(const std::string& name)
  : BT::ActionNodeBase(name, {}),
    nh_("~"),
    cinf_sub_(nh_, "/camera/rgb/camera_info", 1),
    bbx_sub_(nh_, "/darknet_ros/bounding_boxes", 1),
    sync_bbx_(MySyncPolicy_bbx(10), cinf_sub_, bbx_sub_),
    positioned_(false)
  {
    pub_ = nh_.advertise<geometry_msgs::Twist>("mobile_base/commands/velocity", 1);
    sync_bbx_.registerCallback(boost::bind(&RotateUPerson::callback_bbx, this, _1, _2));
  }

  void
  RotateUPerson::halt()
  {
    ROS_INFO("RotateUPerson halt");
  }
  void
  RotateUPerson::callback_bbx(const sensor_msgs::CameraInfoConstPtr& cinf, const darknet_ros_msgs::BoundingBoxesConstPtr& boxes)
  { 
    for (const auto & box : boxes->bounding_boxes) {
        if (box.Class == "person"){
            ROS_INFO("person detected");
            int px = (box.xmax + box.xmin) / 2;
            if (px > cinf->width/2 - cinf->width/8 && px < cinf->width/2 - cinf->width/8 ){
                ROS_INFO("inside the params");
                positioned_ = true;
            }
        }
    }
  }



  BT::NodeStatus
  RotateUPerson::tick()
  {
    if (positioned_)
    {
        ROS_INFO("positioned");
        twist.angular.z=0;
        twist.linear.x=0;
        return BT::NodeStatus::SUCCESS;
    }
    else
    {
        ROS_INFO("rotate");
        twist.angular.z=0.4;
        twist.linear.x=0;
        return BT::NodeStatus::RUNNING;
    }
  }
}  // namespace find_my_mate


#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<find_my_mate::RotateUPerson>("RotateUPerson");
}
