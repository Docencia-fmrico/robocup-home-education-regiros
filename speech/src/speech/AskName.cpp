#include "speech/AskName.h"

namespace speech
{
  AskName::AskName(const std::string& name, const BT::NodeConfiguration& config)
  : BT::ActionNodeBase(name, config),
    nh_("~"),
    detected_(false)
  {
    sub_param = nh_.subscribe("/speech/param", 1, &AskName::nameCallback, this);
  }

  void
  AskName::halt()
  {
    ROS_INFO("AskName halt");
  }

  BT::NodeStatus
  AskName::tick()
  {
    forwarder.speak("Avoid error");
    forwarder.speak("What is your name?");

    forwarder.listen();

    if (detected_){
      detected_ = false;
      return BT::NodeStatus::SUCCESS;
    }
    else {
      return BT::NodeStatus::RUNNING;
    }
  }

  void
  AskName::nameCallback(const std_msgs::StringConstPtr& msg)
  {
    ROS_INFO("PARAMETRO: %s", msg->data.c_str());
    info_.name = msg->data;
    BT::TreeNode::setOutput("Info", info_);
    detected_ = true;
  }

};  // namespace speech

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<speech::AskName>("AskName");
}