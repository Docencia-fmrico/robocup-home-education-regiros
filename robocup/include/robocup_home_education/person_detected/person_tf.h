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

#ifndef ROBOCUP_HOME_PERSON_TF_H
#define ROBOCUP_HOME_PERSON_TF_H

#include <opencv2/core/types.hpp>

#include <string>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/octree/octree_search.h>
#include <pcl/octree/octree.h>
#include <pcl/io/ply_io.h>
#include <pcl_ros/transforms.h>

#include <cv_bridge/cv_bridge.h>

#include <sensor_msgs/Image.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/PointCloud2.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include <boost/algorithm/string.hpp>
#include "msgs/pos_person.h"
#include "ros/ros.h"

namespace robocup_home_education
{

class PersonTf
{
public:
  ///camera/depth/points -> para simulador
  PersonTf();

  void
  cb_person(const msgs::pos_person::ConstPtr& person);

  void
  cloudCB(const sensor_msgs::PointCloud2::ConstPtr& cloud_in);

private:
  ros::NodeHandle nh_;

  ros::Subscriber sub_pos;
  ros::Subscriber cloud_sub_;
  msgs::pos_person pos_person;

  float person_x;
  float person_y;
  bool detected_;

  tf::MessageFilter<sensor_msgs::PointCloud2>* tfPointCloudSub_;
  message_filters::Subscriber<sensor_msgs::PointCloud2>* pointCloudSub_;

  tf::TransformBroadcaster tfBroadcaster_;
  tf::TransformListener tfListener_;

  std::string objectFrameId_;
  std::string workingFrameId_;
  std::string cameraTopicId_;
};

}// namespace robocup_home_education

#endif  //ROBOCUP_HOME_EDUCATION_PERSON_TF_H