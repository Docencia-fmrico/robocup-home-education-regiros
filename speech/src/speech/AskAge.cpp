#include "speech/AskAge.h"

namespace speech
{
  AskAge::AskAge(const std::string& name, const BT::NodeConfiguration& config)
  : BT::ActionNodeBase(name, config),
    nh_("~"),
  {}

  void
  AskAge::halt()
  {
    ROS_INFO("AskAge halt");
  }

  BT::NodeStatus
  AskAge::tick()
  {
    Chat forwarder;
    ros::Subscriber sub_param = nh_.subscribe("/speech/param", 1, startCallback);
    PInfo info_;

    forwarder.speak("Avoid error");
    forwarder.speak("How old are you?");

    forwarder.listen();

    return BT::NodeStatus::RUNNING;

  }

  BT::NodeStatus
  AskAge::ageCallback(const std_msgs::StringConstPtr& msg)
  {
    int age;
    ROS_INFO("PARAMETRO: %s", msg->data.c_str());
    age = stoi(msg->data);
    BT::TreeNode::getInput("Info", info_);
    if (age >= 50) {
      info_.old = true;
    }
    BT::TreeNode::setOutput("Info", info_);
    return BT::NodeStatus::SUCCESS;
  }

};  // namespace speech

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<speech::AskAge>("AskAge");
}