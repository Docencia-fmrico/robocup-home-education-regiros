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

#include <std_msgs/Float32.h>
#include <sensor_msgs/PointCloud2.h>

#include <boost/algorithm/string.hpp>

#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <cv_bridge/cv_bridge.h>

#include <sensor_msgs/Image.h>
#include <darknet_ros_msgs/BoundingBoxes.h>


class Persontf
{
public:
  Persontf(): //objectFrameId_("/person/0")
  //, workingFrameId_("/base_footprint")
  //, cameraTopicId_("/cloud_filtered/0")
  depth_sub_(nh_, "/camera/depth/image_raw", 1)//DARKNET
  , bbx_sub_(nh_, "/darknet_ros/bounding_boxes", 1)//DARKNET
  , sync_bbx_(MySyncPolicy_bbx(10), depth_sub_, bbx_sub_)
  , x_bbx_(0)
  , y_bbx_(0)
  , depth_(0)
  , detected_(false)
  {
    //ros::NodeHandle private_nh("~");
    
    //private_nh.param("object_id", objectFrameId_, objectFrameId_);
    //private_nh.param("cloud_id", cameraTopicId_, cameraTopicId_);

    //ROS_INFO("object_id: [%s]", objectFrameId_.c_str());
    //ROS_INFO("cloud_id : [%s]", cameraTopicId_.c_str());
    pos_pub = nh_.advertise<robocup_home_education::pos_person>("robocup_home/Position_human", 1);
    sync_bbx_.registerCallback(boost::bind(&Persontf::callback_bbx, this, _1, _2));
    //pointCloudSub_ =
    //  new message_filters::Subscriber<sensor_msgs::PointCloud2> (nh_, cameraTopicId_, 5);
    //tfPointCloudSub_ =
    //  new tf::MessageFilter<sensor_msgs::PointCloud2> (*pointCloudSub_, tfListener_, workingFrameId_, 5);
    //tfPointCloudSub_->registerCallback(boost::bind(&Persontf::cloudCB, this, _1));
  }

  /*void cloudCB(const sensor_msgs::PointCloud2::ConstPtr& cloud_in)
  {
    sensor_msgs::PointCloud2 cloud;
    std::cerr << "ENTRO AL CALLBACK TF" << std::endl;
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

    //float x, y, z;
    //int c = 0;
    //x = x = z = 0.0;
   // int n;

    //pcl::PointCloud<pcl::PointXYZRGB>::iterator it;
    
    //pcl::PointXYZRGB p1 = pcrgb->at(y_bbx_, x_bbx_);
    auto point = pcrgb->at(y_bbx_, x_bbx_);

    if(!std::isnan(point.x)) {
      std::cerr << "person at " << depth_ << " in pixel " << x_bbx_ << std::endl;
      std::cerr << "( " << point.x << ", " << point.y << ", " << point.z << " )" << std::endl;
    }
    for (it=pcrgb->begin(); it != pcrgb->end(); ++it)
    {
      if (!std::isnan(it->x) && !std::isnan(it->y) && !std::isnan(it->z))
      {
        x += it->x;
        y += it->y;
        z += it->z;
        c++;
      }
    }

    if (c != 0)
    {
      x = x/c;
      y = y/c;
      z = z/c;


      tf::StampedTransform transform;
      transform.setOrigin(tf::Vector3(x, y, z));
      transform.setRotation(tf::Quaternion(0.0, 0.0, 0.0, 1.0));

      transform.stamp_ = ros::Time::now();
      transform.frame_id_ = workingFrameId_;
      transform.child_frame_id_ = objectFrameId_;

      try
      {
        tfBroadcaster_.sendTransform(transform);
      }
      catch(tf::TransformException& ex)
      {
        ROS_ERROR_STREAM("Transform error of sensor data: " << ex.what() << ", quitting callback");
        return;
      }
    }
    
    if(detected_ && !std::isnan(point.x)) {
      tf::StampedTransform transform;
      transform.setOrigin(tf::Vector3(point.x, point.y, point.z));
      transform.setRotation(tf::Quaternion(0.0, 0.0, 0.0, 1.0));

      transform.stamp_ = ros::Time::now();
      transform.frame_id_ = workingFrameId_;
      transform.child_frame_id_ = objectFrameId_;

      try
      {
        tfBroadcaster_.sendTransform(transform);
      }
      catch(tf::TransformException& ex)
      {
        ROS_ERROR_STREAM("Transform error of sensor data: " << ex.what() << ", quitting callback");
        return;
      }  
    } else {
      ROS_INFO("NO DETECTADO");
    }
  }*/
  
  void
  callback_bbx(const sensor_msgs::ImageConstPtr& depth, const darknet_ros_msgs::BoundingBoxesConstPtr& boxes)
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

    detected_ = false;
    for (const auto & box : boxes->bounding_boxes)
    {
      x_bbx_ = (box.xmax + box.xmin) / 2;
      y_bbx_ = (box.ymax + box.ymin) / 2;

      detected_ = (box.Class == "person");

      if (detected_)
      {
        depth_ = img_ptr_depth->image.at<float>(cv::Point(x_bbx_, y_bbx_)) * 1.0f;
        break;
      }
    }
    if (depth_ > 4.0 || std::isnan(depth_) || std::isinf(depth_))
    {
      detected_ = false;
    }
    /*if (detected_)
    {
      std::cerr << "person at " << depth_ << " in pixel " << x_bbx_ << std::endl;
    }*/
  }


private:
  ros::NodeHandle nh_;
  //Publicador del nuevo topic
  ros::Publisher pos_pub;

  //PARA DARKNET
  message_filters::Subscriber<sensor_msgs::Image> depth_sub_;
  message_filters::Subscriber<darknet_ros_msgs::BoundingBoxes> bbx_sub_;
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, darknet_ros_msgs::BoundingBoxes> MySyncPolicy_bbx;
  message_filters::Synchronizer<MySyncPolicy_bbx> sync_bbx_;
  int x_bbx_;
  int y_bbx_;
  int depth_;
  bool detected_;
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
    //float64 timestamp
  /*
  http://wiki.ros.org/ROS/Tutorials/CreatingMsgAndSrv#Creating_a_msg
  //

    <build_depend>message_generation</build_depend>

    <run_depend>message_runtime</run_depend>
  //Necesario para crear nuevo mensaje

  tf::TransformBroadcaster tfBroadcaster_;
  tf::TransformListener tfListener_;

  tf::MessageFilter<sensor_msgs::PointCloud2>* tfPointCloudSub_;
  message_filters::Subscriber<sensor_msgs::PointCloud2>* pointCloudSub_;

  std::string objectFrameId_;
  
  std::string cameraTopicId_;
  */
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "person_tf");
  Persontf persontf;
  ros::spin();
  return 0;
}
