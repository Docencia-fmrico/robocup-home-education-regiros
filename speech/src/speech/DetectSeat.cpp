#include "speech/DetectSeat.h"

namespace speech
{
  DetectSeat::DetectSeat(const std::string& name)
  : BT::ActionNodeBase(name, {}),
    nh_("~"),
    positioned_(false),
    firsttick_(true),
    cinf_sub_(nh_, "/camera/rgb/camera_info", 1),
    bbx_sub_(nh_, "/darknet_ros/bounding_boxes", 1),
    sync_bbx_(MySyncPolicy_bbx(10), cinf_sub_, bbx_sub_),
    dir_(1)
  {
    pub_ = nh_.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1);
    sync_bbx_.registerCallback(boost::bind(&DetectSeat::callback_chair, this, _1, _2));
  }

  void
  DetectSeat::halt()
  {
    ROS_INFO("DetectSeat halt");
  }


  void DetectSeat::callback_chair(const sensor_msgs::CameraInfoConstPtr& cinf, const darknet_ros_msgs::BoundingBoxesConstPtr& boxes)
  {
    for (const auto & box : boxes->bounding_boxes) {
      if (box.Class == "chair" && box.probability > 0.6){
            ROS_INFO("chair detected");
            int px = (box.xmax + box.xmin) / 2;
            if (px > cinf->width/2 - cinf->width/8 && px < cinf->width/2 + cinf->width/8 ){
                ROS_INFO("inside the params");
                positioned_ = true;
            }
            if (px > cinf->width/2 + cinf->width/8 ){
                ROS_INFO("right");
                dir_ = -1;
            }
            if (px < cinf->width/2 - cinf->width/8 ){
                ROS_INFO("left");
                dir_ = 1;
            }
      }
    }
  }

  BT::NodeStatus
  DetectSeat::tick()
  {
    if (firsttick_){
      initTime_ = ros::Time::now();
      firsttick_ = false;
    }

    if (positioned_){
        ROS_INFO("positioned");
        twist.angular.z=0;
        twist.linear.x=0;
        firsttick_ = true;
        positioned_ = false;
        pub_.publish(twist);
        return BT::NodeStatus::SUCCESS;
    } else if ((ros::Time::now() - initTime_).sec <= 20){
        ROS_INFO("rotate");
        twist.angular.z=0.1*dir_;
        twist.linear.x=0;
        pub_.publish(twist);
        return BT::NodeStatus::RUNNING;
    } else {
        ROS_INFO("time out");
        twist.angular.z=0;
        twist.linear.x=0;
        pub_.publish(twist);
        firsttick_ = true;
        return BT::NodeStatus::SUCCESS;
    }
  }

};  // namespace speech

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<speech::DetectSeat>("DetectSeat");
}