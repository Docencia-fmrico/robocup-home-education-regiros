#include "behaviortree_cpp_v3/behavior_tree.h"
#include "speech/AskDrink.h"

namespace speech
{
  AskDrink::AskDrink(const std::string& name, const BT::NodeConfiguration& config)
  : BT::ActionNodeBase(name, config),
    nh_("~"),
  {}

  void
  AskDrink::halt()
  {
    ROS_INFO("AskDrink halt");
  }

  BT::NodeStatus
  AskDrink::tick()
  {
    Chat forwarder;
    ros::Subscriber sub_param = nh_.subscribe("/speech/param", 1, startCallback);
    PInfo info_;

    forwarder.speak("Avoid error");
    forwarder.speak("What is your favourite drink?");

    forwarder.listen();

    return BT::NodeStatus::RUNNING;

  }

  BT::NodeStatus
  AskDrink::drinkCallback(const std_msgs::StringConstPtr& msg)
  {
    ROS_INFO("PARAMETRO: %s", msg->data.c_str());
    BT::TreeNode::getInput("Info", info_);
    info_.drink;
    BT::TreeNode::setOutput("Info", info_);
    return BT::NodeStatus::SUCCESS;
  }

};  // namespace speech

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<speech::AskDrink>("AskDrink");
}