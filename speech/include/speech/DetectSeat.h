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

#ifndef SPEECH_DETECTSEAT_H
#define SPEECH_DETECTSEAT_H

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "darknet_ros_msgs/BoundingBoxes.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/CameraInfo.h"
#include "geometry_msgs/Twist.h"

#include <iostream>
#include <chrono>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include "string"

namespace speech
{
class DetectSeat : public BT::ActionNodeBase
{
  public:
    explicit DetectSeat(const std::string& name);

    void callback_chair(const sensor_msgs::CameraInfoConstPtr& cinf, const darknet_ros_msgs::BoundingBoxesConstPtr& boxes);

    void halt();

    BT::NodeStatus tick();

  private:
    ros::NodeHandle nh_;
    message_filters::Subscriber<sensor_msgs::CameraInfo> cinf_sub_;
    message_filters::Subscriber<darknet_ros_msgs::BoundingBoxes> bbx_sub_;

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::CameraInfo, darknet_ros_msgs::BoundingBoxes> MySyncPolicy_bbx;
    message_filters::Synchronizer<MySyncPolicy_bbx> sync_bbx_;
    
    geometry_msgs::Twist twist;
    ros::Publisher pub_;
    int count_;
    bool detected_;
    bool positioned_;
    int dir_;
    bool firsttick_;
    ros::Time initTime_;
};
};  // namespace speech

#endif  // SPEECH_DETECTSEAT_H
