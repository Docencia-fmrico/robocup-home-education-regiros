#include "ros/ros.h"
#include "speech/Chat.cpp"

void messageCallback(const std_msgs::StringConstPtr& msg)
{
  ROS_INFO("PARAMETRO: %s", msg->data.c_str());
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "speech_node");
  ros::NodeHandle nh_;
  speech::Chat forwarder;
  ros::Subscriber sub_param = nh_.subscribe("/speech/param", 1, messageCallback);

  forwarder.speak("Avoid error");

  forwarder.speak("Which one is your luggage?");

  forwarder.listen();

  ros::spin();
  return 0;
}