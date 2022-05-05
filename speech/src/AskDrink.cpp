#include "speech/AskDrink.h"
#include "speech/Chat.cpp"
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "ros/ros.h"


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

    forwarder.speak("Avoid error");
    forwarder.speak("I'm ready, tell me when you want me to start");

    forwarder.listen();

    return BT::NodeStatus::RUNNING;

  }

  BT::NodeStatus
  AskDrink::drinkCallback(const std_msgs::StringConstPtr& msg)
  {
    ROS_INFO("PARAMETRO: %s", msg->data.c_str());
    BT::TreeNode::setOutput("FavDrink", msg->data);
    return BT::NodeStatus::SUCCESS;
  }

};  // namespace speech

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<speech::AskDrink>("AskDrink");
}