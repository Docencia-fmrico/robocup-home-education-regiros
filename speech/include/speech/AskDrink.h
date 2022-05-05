#ifndef SPEECH_ASKDRINK_H
#define SPEECH_ASKDRINK_H

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "speech/PersonInfo.h"
#include "speech/Chat.cpp"


namespace speech
{
class AskDrink : public BT::ActionNodeBase
{
  public:
    explicit AskDrink(const std::string& name, const BT::NodeConfiguration& config);

    void halt();

    BT::NodeStatus tick();

    void drinkCallback(const std_msgs::StringConstPtr& msg);

    static PortsList providedPorts()
    {
      return { BidirectionalPort<PInfo>("Info")};
    }
  
  private:
    Chat forwarder;
    PInfo info_;
    ros::NodeHandle nh_;
};
}; //  namespace speech

#endif  // ASKDRINK_H