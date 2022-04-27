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
/*
#include <string>

#include <ros/ros.h>
#include <ros/console.h>


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

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>

#include <boost/algorithm/string.hpp>

*/
#include "string"

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <cv_bridge/cv_bridge.h>

#include <sensor_msgs/Image.h>
#include <darknet_ros_msgs/BoundingBoxes.h>

#include <std_msgs/Float32.h>
#include <sensor_msgs/PointCloud2.h>
#include "msgs/pos_person.h"
#include "ros/ros.h"

class Persontf
{
public:
  Persontf(): depth_sub_(nh_, "/camera/depth/image_raw", 1)//DARKNET
  , bbx_sub_(nh_, "/darknet_ros/bounding_boxes", 1)//DARKNET
  , sync_bbx_(MySyncPolicy_bbx(10), depth_sub_, bbx_sub_)
  , depth_(0)
  {
    pos_pub = nh_.advertise<msgs::pos_person>("/robocup_home/Position_human", 1);
    sync_bbx_.registerCallback(boost::bind(&Persontf::callback_bbx, this, _1, _2));
  }

  void
  callback_bbx(const sensor_msgs::ImageConstPtr& depth, const darknet_ros_msgs::BoundingBoxesConstPtr& boxes)
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


private:
  ros::NodeHandle nh_;
  //Publicador del nuevo topic
  ros::Publisher pos_pub;
  int depth_;
  //PARA DARKNET
  message_filters::Subscriber<sensor_msgs::Image> depth_sub_;
  message_filters::Subscriber<darknet_ros_msgs::BoundingBoxes> bbx_sub_;
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, darknet_ros_msgs::BoundingBoxes> MySyncPolicy_bbx;
  message_filters::Synchronizer<MySyncPolicy_bbx> sync_bbx_;
  
  //std::string workingFrameId_;
  //Hay que pasar la posicion x e y de la persona a través de un topic creado a un node que
  //calcule tf y las publique, mediante otro nodo recibimos las tf y las establecemos como goal
  //en el nodo de navegacion que es el move
  //Revisar el timestamp para ver si la posicion de persona es demasiado antigua
  // el timestamp no debe superar el segundo.
  //Mensaje del topic
  //std_msgs/Header header
      //time stamp
    //bool detected_object
    //int32  p_x
    //int32  p_y
  
  //

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "person_box");
  Persontf persontf;
  ros::spin();
  return 0;
}
