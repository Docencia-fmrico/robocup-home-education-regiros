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

#include "robocup_home_education/IfPerson.h"
#include "robocup_home_education/str_followobj.h"

#include "behaviortree_cpp_v3/behavior_tree.h"


#include "ros/ros.h"

namespace robocup_home_education
{
  IfPerson::IfPerson()
  : depth_sub_(nh_, "/camera/depth/image_raw", 1),
    bbx_sub_(nh_, "/darknet_ros/bounding_boxes", 1),
    //cameraTopicId_("/cloud_filtered/0"),
    sync_bbx_(MySyncPolicy_bbx(10), depth_sub_, bbx_sub_)
  {
    sync_bbx_.registerCallback(boost::bind(&IfPerson::callback_bbx, this, _1, _2));
    //private_nh.param("cloud_id", cameraTopicId_, cameraTopicId_);
   // pointCloudSub_ =
    //  new message_filters::Subscriber<sensor_msgs::PointCloud2> (nh_, cameraTopicId_, 5);
  }

  void
  IfPerson::callback_bbx(const sensor_msgs::ImageConstPtr& depth, const darknet_ros_msgs::BoundingBoxesConstPtr& boxes)
  {
    cv_bridge::CvImagePtr img_ptr_depth;

    try
    {
      img_ptr_depth = cv_bridge::toCvCopy(*depth, sensor_msgs::image_encodings::TYPE_32FC1);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception:  %s", e.what());
      return;
    }

    person.detected = false;

    for (const auto & box : boxes->bounding_boxes)
    {
      person.x = (box.xmax + box.xmin) / 2;
      person.y = (box.ymax + box.ymin) / 2;

      person.detected = (box.Class == "person");

      if (person.detected)
      {
        person.depth = img_ptr_depth->image.at<float>(cv::Point(person.x, person.y)) * 1.0f;
        
        break;
      }
    }
    if (person.depth > 4.0 || std::isnan(person.depth) || std::isinf(person.depth))
    {
      person.detected = false;
    }
    if (person.detected)
    {
      std::cerr << "person at " << person.depth << " in pixel " << person.x << std::endl;
    }
  }

}  // namespace robocup_home_education
/*Topics de darknet ros:
/darknet_ros/bounding_boxes
/darknet_ros/check_for_objects/cancel
/darknet_ros/check_for_objects/feedback
/darknet_ros/check_for_objects/goal
/darknet_ros/check_for_objects/result
/darknet_ros/check_for_objects/status
/darknet_ros/detection_image
/darknet_ros/found_object
*/