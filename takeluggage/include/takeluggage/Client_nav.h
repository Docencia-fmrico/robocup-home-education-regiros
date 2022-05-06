#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "takeluggage/str_followobj.h"

#include "ros/ros.h"

#ifndef TAKELUGGAGE_CLIENT_NAV_H
#define TAKELUGGAGE_CLIENT_NAV_H

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> Client;

namespace takeluggage
{
class GoPoint: public BT::ActionNodeBase
{
public:
	GoPoint(const std::string& name, const BT::NodeConfiguration& config);

	BT::NodeStatus tick();

	void halt();
    
	void doWork(long int until);

	void feedbackCb(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback);

	void doneCb(const actionlib::SimpleClientGoalState& state,
			const move_base_msgs::MoveBaseResultConstPtr& result);

    static BT::PortsList providedPorts()
    {
      return { BT::InputPort<PointTF>("point")};
    }

private:
	Client ac;
	bool sent;
	bool finished;
	PointTF p;
};
}

#endif