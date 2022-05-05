#ifndef FIND_MY_MATE_GETINFO_H
#define FIND_MY_MATE_GETINFO_H

#include "string"

#include <sensor_msgs/Image.h>
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "darknet_ros_msgs/BoundingBoxes.h"
#include "color_filter/colorpart.h"
#include "find_my_mate/str_info.h"
#include "find_my_mate/Chat.h"


#include "ros/ros.h"

#include <std_msgs/Float32.h>

namespace find_my_mate
{
class GetInfo : public BT::ActionNodeBase
{
  public:
    explicit GetInfo(const std::string& name, const BT::NodeConfiguration& config);

    void halt();

    BT::NodeStatus tick();

    void callback_clrpart(const color_filter::colorpartConstPtr& clrpart);

    void callback_obj(const darknet_ros_msgs::BoundingBoxesConstPtr& boxes);

    void callback_name(const std_msgs::StringConstPtr& msg);

    static BT::PortsList providedPorts()
    {
      return { BT::OutputPort<Infop>("info")};
    }

  private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_clr_;
    ros::Subscriber sub_obj_;
    ros::Subscriber sub_name_;
    find_my_mate::Chat forwarder;
    Infop info_;
    bool firsttick_;
    bool detectedclr_;
    bool detectedobj_;
    bool detectedname_;
    
};
}  // namespace find_my_mate

#endif  // FIND_MY_MATE_GETINFO_H
