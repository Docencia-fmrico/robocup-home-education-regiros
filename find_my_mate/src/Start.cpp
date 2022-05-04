#include "ros/ros.h"
#include "find_my_mate/Start.h"
#include "find_my_mate/Chat.cpp"
#include "behaviortree_cpp_v3/behavior_tree.h"

namespace find_my_mate
{
  Start::Start(const std::string& name, const BT::NodeConfiguration& config)
  : BT::ActionNodeBase(name, config),
    nh_("~"),
  {}

  void
  Start::halt()
  {
    ROS_INFO("Start halt");
  }

  BT::NodeStatus
  GetInfo::tick()
  {
    speech::Chat forwarder;
    ros::Subscriber sub_param = nh_.subscribe("/speech/param", 1, startCallback);

    forwarder.speak("Avoid error");
    forwarder.speak("I'm ready, tell me when you want me to start");

    forwarder.listen();

    return BT::NodeStatus::RUNNING;

  }

  BT::NodeStatus
  startCallback(const std_msgs::StringConstPtr& msg)
  {
    ROS_INFO("PARAMETRO: %s", msg->data.c_str());
    if (msg->data == "start"){
      return BT::NodeStatus::SUCCESS;
    }
  }

};  // namespace find_my_mate

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<find_my_mate::Start>("Start");
}