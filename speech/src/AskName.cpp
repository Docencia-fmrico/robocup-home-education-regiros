namespace speech
{
  AskName::AskName(const std::string& name, const BT::NodeConfiguration& config)
  : BT::ActionNodeBase(name, config),
    nh_("~"),
  {}

  void
  AskName::halt()
  {
    ROS_INFO("AskName halt");
  }

  BT::NodeStatus
  AskName::tick()
  {
    Chat forwarder;
    ros::Subscriber sub_param = nh_.subscribe("/speech/param", 1, startCallback);

    forwarder.speak("Avoid error");
    forwarder.speak("I'm ready, tell me when you want me to start");

    forwarder.listen();

    return BT::NodeStatus::RUNNING;

  }

  BT::NodeStatus
  AskName::nameCallback(const std_msgs::StringConstPtr& msg)
  {
    ROS_INFO("PARAMETRO: %s", msg->data.c_str());
    return BT::NodeStatus::SUCCESS;
  }

};  // namespace speech

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<speech::AskName>("AskName");
}