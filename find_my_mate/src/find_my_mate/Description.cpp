#include "find_my_mate/Description.h"

namespace find_my_mate
{

  Description::Description(const std::string& name, const BT::NodeConfiguration& config)
  : BT::ActionNodeBase(name, config),
    nh_("~")
  {}


  void
  Description::halt()
  {
    ROS_INFO("Description halt");
  }

  BT::NodeStatus 
  Description::tick()
  {
    BT::TreeNode::getInput("info", info_);
    std::ostringstream oss;
    std::string phrase;

    oss << "Their name is " << info_.name << ", their shirt is " << info_.color << ", and they were holding a " << info_.object;
    phrase = oss.str();

    forwarder.speak("Avoid error");
    forwarder.speak(phrase);
  }

}  // namespace find_my_mate

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<find_my_mate::Description>("Description");
}