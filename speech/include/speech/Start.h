#ifndef SPEECH_START_H
#define SPEECH_START_H

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "speech/Chat.h"

namespace speech
{
class Start : public BT::ActionNodeBase
{
  public:
    explicit Start(const std::string& name);

    void halt();

    BT::NodeStatus tick();

    void startCallback(const std_msgs::StringConstPtr& msg);
  
  private:
    Chat forwarder;
    ros::NodeHandle nh_;
    ros::Subscriber sub_param;
    bool detected_;
};
}; //  speech

#endif  // SPEECH_H