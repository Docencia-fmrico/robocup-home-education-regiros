#ifndef SPEECH_OFFERSEAT_H
#define SPEECH_OFFERSEAT_H

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "speech/PersonInfo.h"
#include "speech/Chat.cpp"
#include "string"

namespace speech
{
class OfferSeat : public BT::ActionNodeBase
{
  public:
    explicit OfferSeat(const std::string& name, const BT::NodeConfiguration& config);

    void halt();

    BT::NodeStatus tick();

    static PortsList providedPorts()
    {
      return {InputPort<PInfo>("Info")};
    }

  private:
    Chat forwarder;
    PInfo info_;
    ros::NodeHandle nh_;
};
}; //  namespace speech

#endif  // SPEECH_OFFERSEAT_H