#ifndef SPEECH_DETECTSEAT_H
#define SPEECH_DETECTSEAT_H

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "string"

namespace speech
{
class DetectSeat : public BT::ActionNodeBase
{
  public:
    explicit DetectSeat(const std::string& name);

    void halt();

    BT::NodeStatus tick();

  private:
    ros::NodeHandle nh_;
};
}; //  namespace speech

#endif  // SPEECH_DETECTSEAT_H