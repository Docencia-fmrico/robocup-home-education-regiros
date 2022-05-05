#include "speech/OfferSeat.h"

namespace speech
{
  OfferSeat::OfferSeat(const std::string& name, const BT::NodeConfiguration& config)
  : BT::ActionNodeBase(name, config),
    nh_("~")
  {}

  void
  OfferSeat::halt()
  {
    ROS_INFO("OfferSeat halt");
  }

  BT::NodeStatus
  OfferSeat::tick()
  {
    BT::TreeNode::getInput("Info", info_);

    forwarder.speak("Avoid error");

    if (info_.old) {
      forwarder.speak("Please sit in the sofa");
    }
    else {
      forwarder.speak("You can seat in this chair if you want");
    }

    return BT::NodeStatus::SUCCESS;
  }

};  // namespace speech

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<speech::OfferSeat>("OfferSeat");
}