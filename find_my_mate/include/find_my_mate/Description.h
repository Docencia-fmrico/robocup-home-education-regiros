#ifndef FIND_MY_MATE_ASKNAME_H
#define FIND_MY_MATE_ASKNAME_H

#include "string"

#include <sensor_msgs/Image.h>
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "color_filter/colorpart.h"

#include "ros/ros.h"

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <std_msgs/Float32.h>

namespace find_my_mate
{
class AskName : public BT::ActionNodeBase
{
  public:
    explicit AskName(const std::string& name, const BT::NodeConfiguration& config);

    void halt();

    BT::NodeStatus tick();

    void callback(const color_filter::colorpartConstPtr& clrpart);//callback si se necesita

    static BT::PortsList providedPorts()
    {
      return { BT::OutputPort<std::string>("name")}; //cambia el tipo que salga
    }

  private:
    ros::NodeHandle nh_;
    bool firstick;
    bool detected;
    
};
}  // namespace find_my_mate

#endif  // FIND_MY_MATE_ASKNAME_H