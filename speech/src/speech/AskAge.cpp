#include "speech/AskAge.h"

namespace speech
{
  AskAge::AskAge(const std::string& name, const BT::NodeConfiguration& config)
  : BT::ActionNodeBase(name, config),
    nh_("~")
  {
    sub_param = nh_.subscribe("/speech/param", 1, &AskAge::ageCallback, this);
  }

  void
  AskAge::halt()
  {
    ROS_INFO("AskAge halt");
  }

  BT::NodeStatus
  AskAge::tick()
  {
    forwarder.speak("Avoid error");
    forwarder.speak("How old are you?");

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
    detected_ = true;
  }

};  // namespace speech

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<speech::AskAge>("AskAge");
}