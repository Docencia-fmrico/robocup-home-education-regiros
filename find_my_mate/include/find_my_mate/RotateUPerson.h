#ifndef FIND_MY_MATE_ROTATEUPERSON_H
#define FIND_MY_MATE_ROTATEUPERSON_H

#include "string"

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#include "darknet_ros_msgs/BoundingBoxes.h"
#include "find_my_mate/str_info.h"
#include "sensor_msgs/CameraInfo.h"
#include "geometry_msgs/Twist.h"

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include "ros/ros.h"

namespace find_my_mate
{
class RotateUPerson : public BT::ActionNodeBase
{
  public:
    explicit RotateUPerson(const std::string& name);

    void halt();

    BT::NodeStatus tick();

    void callback_bbx(const sensor_msgs::CameraInfoConstPtr&, const darknet_ros_msgs::BoundingBoxesConstPtr& boxes);

  private:
    ros::NodeHandle nh_;
    message_filters::Subscriber<sensor_msgs::CameraInfo> cinf_sub_;
    message_filters::Subscriber<darknet_ros_msgs::BoundingBoxes> bbx_sub_;

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::CameraInfo, darknet_ros_msgs::BoundingBoxes> MySyncPolicy_bbx;
    message_filters::Synchronizer<MySyncPolicy_bbx> sync_bbx_;
    
    geometry_msgs::Twist twist;
    ros::Publisher pub_;

    bool positioned_;
    
};
}  // namespace find_my_mate

#endif  // FIND_MY_MATE_ROTATEUPERSON_H
