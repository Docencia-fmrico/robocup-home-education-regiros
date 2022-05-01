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
/*
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
#include <message_filters/subscriber.h>
#include <boost/algorithm/string.hpp>
#include "msgs/pos_person.h"
#include "ros/ros.h"
*/
#include "robocup_home_education/person_detected/person_tf.h"

namespace robocup_home_education
{
  ///camera/depth_registered/points -> topic camara clase
  PersonTf::PersonTf(): objectFrameId_("/person/0")
  , workingFrameId_("/base_footprint")
  , cameraTopicId_("/cloud_filtered/0")
  {
    ros::NodeHandle private_nh("~");
    private_nh.param("person_id", objectFrameId_, objectFrameId_);
    private_nh.param("cloud_id", cameraTopicId_, cameraTopicId_);

    ROS_INFO("person_id: [%s]", objectFrameId_.c_str());
    ROS_INFO("cloud_id : [%s]", cameraTopicId_.c_str());
    sub_pos = nh_.subscribe("/robocup_home/Position_human", 1, &PersonTf::cb_person, this);
    cloud_sub_ = nh_.subscribe("/camera/depth/points", 1, &PersonTf::cloudCB, this);
  }

  void
  PersonTf::cb_person(const msgs::pos_person::ConstPtr& person)
  {
    ROS_INFO("ENTRO AL PERSONCB");
    person_x = person->p_x;
    person_y = person->p_y;
    detected_ = person->detected_;
  }

  void
  PersonTf::cloudCB(const sensor_msgs::PointCloud2::ConstPtr& cloud_in)
  {
    sensor_msgs::PointCloud2 cloud;
    
    try
    {
      pcl_ros::transformPointCloud(workingFrameId_, *cloud_in, cloud, tfListener_);
    }
    catch(tf::TransformException & ex)
    {
      ROS_ERROR_STREAM("Transform error of sensor data: " << ex.what() << ", quitting callback");
      return;
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcrgb(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromROSMsg(cloud, *pcrgb);

    auto point = pcrgb->at(person_x, person_y);
    
    if((!std::isnan(point.x) || !std::isinf(point.x)) && detected_) {
      ROS_INFO("ENTRO AL CLOUDCB");
      tf::StampedTransform transform_;
      transform_.setOrigin(tf::Vector3(point.x, point.y, point.z));
      transform_.setRotation(tf::Quaternion(0.0, 0.0, 0.0, 1.0));

      transform_.stamp_ = ros::Time::now();
      transform_.frame_id_ = workingFrameId_;
      transform_.child_frame_id_ = objectFrameId_;
    
      try
      {
        tfBroadcaster_.sendTransform(transform_);
      }
      catch(tf::TransformException& ex)
      {
        ROS_ERROR_STREAM("Transform error of sensor data: " << ex.what() << ", quitting callback");
        return;
      }
    }
  }
}   // namespace robocup_home_education