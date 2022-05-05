#include "speech/DetectSeat.h"

namespace speech
{
  DetectSeat::DetectSeat(const std::string& name, const BT::NodeConfiguration& config)
  : BT::ActionNodeBase(name, config),
    nh_("~"),
  {}

  void
  DetectSeat::halt()
  {
    ROS_INFO("DetectSeat halt");
  }

  BT::NodeStatus
  DetectSeat::tick()
  {
    int count_ = 0;

    while (count < 10){
      count++;
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