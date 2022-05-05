#ifndef SPEECH_MOVE_H
#define SPEECH_MOVE_H

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "string"

namespace speech
{
class Move : public BT::ActionNodeBase
{
  public:
    explicit Move(const std::string& name);

    void halt();

    BT::NodeStatus tick();

  private:
    ros::NodeHandle nh_;
};
}; //  namespace speech

#endif  // SPEECH_MOVE_H