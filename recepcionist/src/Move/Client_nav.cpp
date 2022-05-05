#include "recepcionist/Client_nav.h"


namespace recepcionist
{
	GoPoint::GoPoint(const std::string& name, const BT::NodeConfiguration& config)
	: ac("move_base", true),
	  BT::ActionNodeBase(name, config),
	  sent(true),
	  finished(false)
	{
		ROS_INFO("Waiting for action server to start.");
		ac.waitForServer();
		ROS_INFO("Action server started, sending goal.");
	}

	void 
	GoPoint::doWork(long int until)
	{
		p = getInput<PointTF>("point").value();
		move_base_msgs::MoveBaseGoal goal;
    	std::cerr << "PUNTOS: [" << p.x << ", " << p.y << ", " << p.z << "]" << std::endl;
		goal.target_pose.header.frame_id = "map";
		goal.target_pose.header.stamp = ros::Time::now();
		goal.target_pose.pose.position.x = p.x;
		goal.target_pose.pose.position.y = p.y;
		goal.target_pose.pose.position.z = p.z;
		goal.target_pose.pose.orientation.x = p.or_x;
		goal.target_pose.pose.orientation.y = p.or_y;
		goal.target_pose.pose.orientation.z = p.or_z;
		goal.target_pose.pose.orientation.w = p.or_w;

		ROS_INFO("Sending action");
			ac.sendGoal(goal,
					boost::bind(&GoPoint::doneCb, this, _1, _2),
					Client::SimpleActiveCallback(),
					boost::bind(&GoPoint::feedbackCb, this, _1));

		ROS_INFO("Action sent");
		finished = false;
	}

	void 
	GoPoint::halt()
	{
		ROS_INFO("GoPoint halt");
	}

	BT::NodeStatus 
	GoPoint::tick()
  	{
	  if(finished) {
		  finished = false;
		return BT::NodeStatus::SUCCESS;
	  }

	  if (sent) {
		ROS_INFO("ENVIO TRABAJO");
		doWork(3);
		sent = false;
	  }

	  return BT::NodeStatus::RUNNING; 
	}

	void 
	GoPoint::feedbackCb(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback)
	{
		ROS_INFO("Current count %lf", feedback->base_position.pose.position.x);
	}

	void 
	GoPoint::doneCb(const actionlib::SimpleClientGoalState& state,
			const move_base_msgs::MoveBaseResultConstPtr& result)
	{
		ROS_INFO("Finished in state [%s]", state.toString().c_str());
		sent = true;
		finished = true;
	}

    static BT::PortsList providedPorts()
    {
      return { BT::InputPort<PointTF>("point")};
    }
}

#include "behaviortree_cpp_v3/bt_factory.h"

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<recepcionist::GoPoint>("GoPoint");
}