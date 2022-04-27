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

#include <opencv2/core/types.hpp>

#include "string"
#include <image_geometry/pinhole_camera_model.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <cv_bridge/cv_bridge.h>

#include <sensor_msgs/Image.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/PointCloud2.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>


#include "ros/ros.h"

class PersonTf
{
public:
  
  PersonTf()
  : depth_sub_(nh_, "/camera/depth/image_raw", 1),
    bbx_sub_(nh_, "/darknet_ros/bounding_boxes", 1),
    sync_bbx_(MySyncPolicy_bbx(10), depth_sub_, bbx_sub_),
    objectFrameId_("/person/0"),
    workingFrameId_("/base_footprint")
    {
      sub_cam_ = nh_.subscribe("/camera/depth/camera_info", 1, &PersonTf::cb_camera_info, this);
      sync_bbx_.registerCallback(boost::bind(&PersonTf::cb_tf, this, _1, _2));
    }

  void
  cb_tf(const sensor_msgs::ImageConstPtr& img, const darknet_ros_msgs::BoundingBoxesConstPtr& boxes)
  {
    bool detected_;
    int x;
    int y;
    int z;

    cv_bridge::CvImagePtr img_ptr_depth;

    try
    {
      img_ptr_depth = cv_bridge::toCvCopy(*img, sensor_msgs::image_encodings::TYPE_32FC1);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception:  %s", e.what());
      return;
    }

    detected_ = false;

    for (const auto & box : boxes->bounding_boxes)
    {
      x = (box.xmax + box.xmin) / 2;
      y = (box.ymax + box.ymin) / 2;

      detected_ = (box.Class == "person");

      if (detected_)
      {
        depth_ = img_ptr_depth->image.at<float>(cv::Point(x, y)) * 1.0f;
        
        break;
      }
    }
    if (depth_ > 4.0 || std::isnan(depth_) || std::isinf(depth_))
    {
      detected_ = false;
    }
    if (detected_)
    {
      std::cerr << "person at " << depth_ << " in pixel " << x << std::endl;
      /*
      cv::Point pt(x, y);
      cv::Point3d pt3d(pt.x, pt.y, 0.0);
      //xyz_(x_d, y_d, 0.0);
      xyz_ = pt3d;
      pxl_ = cam_mod_.project3dToPixel(xyz_);
      
      //tf::StampedTransform transform;
      transform_.setOrigin(tf::Vector3(pxl_.x, pxl_.y, 0));
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
      }*/
    } else {
      std::cerr << "PERSON NOT DETECTED" << x << std::endl;
    }
    
  }

  void cb_camera_info(const sensor_msgs::CameraInfoConstPtr& msg)
  {
    cam_mod_.fromCameraInfo(msg);
  }


private:
  ros::NodeHandle nh_;

  tf::TransformBroadcaster tfBroadcaster_;
  tf::StampedTransform transform_;

  cv::Point2d pxl_;
  cv::Point3d xyz_;
  ros::Subscriber sub_cam_;
  image_geometry::PinholeCameraModel cam_mod_;
  double depth_;
  ros::Time time_bound;

  std::string objectFrameId_;
  std::string workingFrameId_;
  
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, darknet_ros_msgs::BoundingBoxes> MySyncPolicy_bbx;
  message_filters::Synchronizer<MySyncPolicy_bbx> sync_bbx_;
  message_filters::Subscriber<sensor_msgs::Image> depth_sub_;
  message_filters::Subscriber<darknet_ros_msgs::BoundingBoxes> bbx_sub_;
  
  
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "person_tf");
  PersonTf persontf;
  ros::Rate loop_rate(20);
  while (ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }
  //ros::spin();
  return 0;
}

/*Mensaje sensor_msgs/CameraInfo del topic /camera/depth/camera_info
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
uint32 height
uint32 width
string distortion_model
float64[] D
float64[9] K
float64[9] R
float64[12] P
uint32 binning_x
uint32 binning_y
sensor_msgs/RegionOfInterest roi
  uint32 x_offset
  uint32 y_offset
  uint32 height
  uint32 width
  bool do_rectify
*/