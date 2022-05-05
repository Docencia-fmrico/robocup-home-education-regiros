#include "ros/ros.h"
#include "find_my_mate/Starting.h"
#include "find_my_mate/Chat.h"
#include "behaviortree_cpp_v3/behavior_tree.h"

namespace find_my_mate
{

  Starting::Starting(const std::string& name)
  : BT::ActionNodeBase(name, {}),
    nh_("~"),
    getparam_(false)
  {
    sub_param_ = nh_.subscribe("/speech/param", 1, &Starting::StartingCallback, this);
  }

  void
  Starting::halt()
  {
    ROS_INFO("Starting halt");
  }

  BT::NodeStatus
  Starting::tick()
  {
    find_my_mate::Chat forwarder;

    forwarder.speak("Avoid error");
    forwarder.speak("I'm ready, tell me when you want me to start");

    forwarder.listen();

    if (getparam_){
      getparam_ = false;
      return BT::NodeStatus::SUCCESS;
    }

    return BT::NodeStatus::RUNNING;
  }

  void
  Starting::StartingCallback(const std_msgs::StringConstPtr& msg)
  {
    ROS_INFO("PARAMETRO: %s", msg->data.c_str());
    if (msg->data == "start"){
      getparam_ = true;
    }
  }

};  // namespace find_my_mate

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<find_my_mate::Starting>("Starting");
}