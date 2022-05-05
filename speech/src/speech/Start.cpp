#include "ros/ros.h"
#include "speech/Start.h"
#include "behaviortree_cpp_v3/behavior_tree.h"

namespace speech
{
  Start::Start(const std::string& name)
  : BT::ActionNodeBase(name, {}),
    nh_("~")
  {
    sub_param = nh_.subscribe("/speech/param", 1, &Start::startCallback, this);
  }

  void
  Start::halt()
  {
    ROS_INFO("Start halt");
  }

  BT::NodeStatus
  Start::tick()
  {
    forwarder.speak("Avoid error");
    forwarder.speak("I'm ready, tell me when you want me to start");

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
  Start::startCallback(const std_msgs::StringConstPtr& msg)
  {
    ROS_INFO("PARAMETRO: %s", msg->data.c_str());
    if (msg->data == "start"){
      detected_ = true;
    }
  }

};  // namespace speech

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<speech::Start>("Start");
}