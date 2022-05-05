#include "speech/DetectSeat.h"

namespace speech
{
  DetectSeat::DetectSeat(const std::string& name)
  : BT::ActionNodeBase(name, {}),
    nh_("~"),
    detected_(false)
  {}

  void
  DetectSeat::halt()
  {
    ROS_INFO("DetectSeat halt");
  }


  void DetectSeat::callback_chair(const darknet_ros_msgs::BoundingBoxesConstPtr& boxes)
  {
    for (const auto & box : boxes->bounding_boxes) {
      if (box.Class == "chair" && box.probability > 0.6){
        
      }
    }
  }

  BT::NodeStatus
  DetectSeat::tick()
  {
    int count_ = 0;

    while (count_ < 10){
      count_++;
      return BT::NodeStatus::RUNNING;
    }

    return BT::NodeStatus::SUCCESS;
  }

};  // namespace speech

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<speech::DetectSeat>("DetectSeat");
}