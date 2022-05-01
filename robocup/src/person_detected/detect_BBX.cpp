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

#include "string"

#include "robocup_home_education/person_detected/detect_BBX.h"

#include "ros/ros.h"

namespace robocup_home_education
{
    
  PersonBBX::PersonBBX(): depth_sub_(nh_, "/camera/depth/image_raw", 1)
  , bbx_sub_(nh_, "/darknet_ros/bounding_boxes", 1)
  , sync_bbx_(MySyncPolicy_bbx(10), depth_sub_, bbx_sub_)
  , depth_(0)
  {
    pos_pub = nh_.advertise<msgs::pos_person>("/robocup_home/Position_human", 1);
    sync_bbx_.registerCallback(boost::bind(&PersonBBX::callback_bbx, this, _1, _2));
  }

  void
  PersonBBX::callback_bbx(const sensor_msgs::ImageConstPtr& depth, const darknet_ros_msgs::BoundingBoxesConstPtr& boxes)
  {
    cv_bridge::CvImagePtr img_ptr_depth;
    msgs::pos_person pos_person;

    try
    {
      img_ptr_depth = cv_bridge::toCvCopy(*depth, sensor_msgs::image_encodings::TYPE_32FC1);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception:  %s", e.what());
      return;
    }

    pos_person.detected_ = false;
    for (const auto & box : boxes->bounding_boxes)
    {
      pos_person.p_x = (box.xmax + box.xmin) / 2;
      pos_person.p_y = (box.ymax + box.ymin) / 2;

      pos_person.detected_ = (box.Class == "person");

      if (pos_person.detected_)
      {
        depth_ = img_ptr_depth->image.at<float>(cv::Point(pos_person.p_x, pos_person.p_y)) * 1.0f;
        std::cerr << pos_person.detected_ << " distance -> (" << depth_ << std::endl;
        break;
      }
    }
    if (depth_ > 4.0 || depth_ < 0.0 || std::isnan(depth_) || std::isinf(depth_))
    {
      pos_person.detected_ = false;
    }
    if (pos_person.detected_)
    {
      std::cerr << "person at " << depth_ << " in pixel " << pos_person.p_x << std::endl;
      pos_pub.publish(pos_person);
    }
  }

}   // namespace robocup_home_education

/*
darknet_ros_msgs/BoundingBox[] bounding_boxes
  float64 probability
  int64 xmin
  int64 ymin
  int64 xmax
  int64 ymax
  int16 id
  string Class

*/