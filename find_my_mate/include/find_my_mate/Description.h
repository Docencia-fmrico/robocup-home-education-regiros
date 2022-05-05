#ifndef FIND_MY_MATE_DESCRIPTION_H
#define FIND_MY_MATE_DESCRIPTION_H

#include "string"
#include <sstream>

#include <sensor_msgs/Image.h>
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "color_filter/colorpart.h"
#include "find_my_mate/str_info.h"
#include "find_my_mate/Chat.h"


#include "ros/ros.h"

#include <std_msgs/Float32.h>

namespace find_my_mate
{
class Description : public BT::ActionNodeBase
{
  public:
    explicit Description(const std::string& name, const BT::NodeConfiguration& config);

    void halt();

    BT::NodeStatus tick();

    static BT::PortsList providedPorts()
    {
      return { BT::InputPort<Infop>("info")}; //cambia el tipo que salga
    }

  private:
    ros::NodeHandle nh_;
    bool detected;
    find_my_mate::Chat forwarder;
    Infop info_;
    
};
}  // namespace find_my_mate

#endif  // FIND_MY_MATE_DESCRIPTION_H