#include "behaviortree_cpp_v3/behavior_tree.h"
#include "speech/AskDrink.h"

namespace speech
{
  AskDrink::AskDrink(const std::string& name, const BT::NodeConfiguration& config)
  : BT::ActionNodeBase(name, config),
    nh_("~")
  {
    sub_param = nh_.subscribe("/speech/param", 1, &AskDrink::drinkCallback, this);
  }

  void
  AskDrink::halt()
  {
    ROS_INFO("AskDrink halt");
  }

  BT::NodeStatus
  AskDrink::tick()
  {
    forwarder.speak("Avoid error");
    forwarder.speak("What is your favourite drink?");

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
  AskDrink::drinkCallback(const std_msgs::StringConstPtr& msg)
  {
    ROS_INFO("PARAMETRO: %s", msg->data.c_str());
    BT::TreeNode::getInput("Info", info_);
    info_.drink;
    BT::TreeNode::setOutput("Info", info_);
    detected_ = true;
  }

};  // namespace speech

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<speech::AskDrink>("AskDrink");
}