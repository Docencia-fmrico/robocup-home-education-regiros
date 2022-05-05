#ifndef SPEECH_ASKAGE_H
#define SPEECH_ASKAGE_H

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "speech/PersonInfo.h"
#include "speech/Chat.h"

namespace speech
{
class AskAge : public BT::ActionNodeBase
{
  public:
    explicit AskAge(const std::string& name, const BT::NodeConfiguration& config);

    void halt();

    BT::NodeStatus tick();

    void ageCallback(const std_msgs::StringConstPtr& msg);

    static BT::PortsList providedPorts()
    {
      return {BT::BidirectionalPort<PInfo>("Info")};
    }
  
  private:
    Chat forwarder;
    PInfo info_;
    ros::NodeHandle nh_;
    ros::Subscriber sub_param;
    bool detected_;
};
}; //  namespace speech

#endif  // ASKAGE_H