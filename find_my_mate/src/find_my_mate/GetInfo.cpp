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

#include "find_my_mate/GetInfo.h"
#include "find_my_mate/str_info.h"
#include "color_filter/colorpart.h"

#include "behaviortree_cpp_v3/behavior_tree.h"

#include "ros/ros.h"

namespace find_my_mate
{
  GetInfo::GetInfo(const std::string& name, const BT::NodeConfiguration & config)
  : BT::ActionNodeBase(name, config),
    nh_("~"),
    firsttick_(true),
    detectedclr_(false),
    detectedname_(false),
    detectedobj_(false)
  { 
    sub_clr_ = nh_.subscribe("/rgbcolor/colorpart", 1, &GetInfo::callback_clrpart, this ) ;
    sub_obj_ = nh_.subscribe("/darknet_ros/bounding_boxes", 1, &GetInfo::callback_obj, this ) ;
  }

  void
  GetInfo::halt()
  {
    ROS_INFO("GetInfo halt");
  }

  void
  GetInfo::callback_clrpart(const color_filter::colorpartConstPtr& clrpart)
  {
    info_.color = clrpart->color.data;
    detectedclr_ = true;
    ROS_INFO("color");
  }

  void
  GetInfo::callback_obj(const darknet_ros_msgs::BoundingBoxesConstPtr& boxes)
  { 
    for (const auto & box : boxes->bounding_boxes) {
        if (box.Class != "person"){
          ROS_INFO("object");
          info_.object = box.Class;
          detectedobj_ = true;
        }
    }
  }



  BT::NodeStatus
  GetInfo::tick()
  {
    if (firsttick_){
        ROS_INFO("speek");
        //iniciar speech.
        firsttick_=false;
        detectedname_=true;
    }


    if (detectedclr_ && detectedname_ && detectedobj_)
    {
      

        ROS_INFO("finishing");
        BT::TreeNode::setOutput("info", info_);
        firsttick_ = true;
        detectedclr_ = false;
        detectedname_ = false;
        detectedobj_ = false;
        return BT::NodeStatus::SUCCESS;
    }
    else
    {
      return BT::NodeStatus::RUNNING;
    }
  }
}  // namespace find_my_mate


#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<find_my_mate::GetInfo>("GetInfo");
}
