#ifndef FIND_MY_MATE_STARTING_H
#define FIND_MY_MATE_STARTING_H

#include "string"
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "ros/ros.h"
#include "std_msgs/String.h"

namespace find_my_mate
{
  class Starting : public BT::ActionNodeBase
{
    public:
      explicit Starting(const std::string& name);

      void halt();

      BT::NodeStatus tick();

      void StartingCallback(const std_msgs::StringConstPtr& msg);
    
    private:
      ros::NodeHandle nh_;
      ros::Subscriber sub_param_;
      bool getparam_ ; 
};
} //  find_my_mate

#endif  // FIND_MY_MATE_STARTING_H

