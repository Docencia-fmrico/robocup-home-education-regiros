#ifndef SPEECH_START_H
#define SPEECH_START_H

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "ros/ros.h"
#include "std_msgs/String.h"

namespace speech
{
class Start : public BT::ActionNodeBase
{
  public:
    explicit Start(const std::string& name, const BT::NodeConfiguration& config);

    void halt();

    BT::NodeStatus tick();

    void startCallback(const std_msgs::StringConstPtr& msg);
    
};
}; //  speech

#endif  // SPEECH_H