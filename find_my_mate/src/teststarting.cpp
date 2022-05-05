#include "ros/ros.h"
#include "find_my_mate/Starting.h"
#include "find_my_mate/Chat.h"
#include "behaviortree_cpp_v3/behavior_tree.h"
#include <string>
#include <sstream>

namespace find_my_mate
{
  void messageCallback(const std_msgs::StringConstPtr& msg)
  {
    ROS_INFO("PARAMETRO: %s", msg->data.c_str());
  }

};  // namespace find_my_mate

int
  main(int argc, char** argv) 
  {
    ros::init(argc, argv, "startingtest_node");
    std::string name = "Jhon";
    std::string color = "Blue";
    std::string object = "Bottle";
    std::ostringstream oss;
    std::string phrase;

    find_my_mate::Chat forwarder ;
    //ros::NodeHandle nh_;
    //ros::Subscriber sub_param = nh_.subscribe("/speech/param", 1, find_my_mate::messageCallback);

    oss << "Their name is " << name << ", their shirt is " << color << ", and they were holding a " << object;
    phrase = oss.str();

    forwarder.speak("Avoid error");
    forwarder.speak(phrase);

    ros::spin();
    return 0;
  }
