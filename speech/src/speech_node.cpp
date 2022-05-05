#include <string>
#include <memory>

#include "ros/ros.h"

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/utils/shared_library.h"
#include "behaviortree_cpp_v3/loggers/bt_zmq_publisher.h"

#include "ros/package.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "speech_node");
  ros::NodeHandle n;

  BT::BehaviorTreeFactory factory;
  BT::SharedLibrary loader;

  factory.registerFromPlugin(loader.getOSName("asr_Start_bt_node"));
  factory.registerFromPlugin(loader.getOSName("asr_AskName_bt_node"));
  factory.registerFromPlugin(loader.getOSName("asr_Move_bt_node"));
  factory.registerFromPlugin(loader.getOSName("asr_AskDrink_bt_node"));
  factory.registerFromPlugin(loader.getOSName("asr_AskAge_bt_node"));
  factory.registerFromPlugin(loader.getOSName("asr_DetectSeat_bt_node"));
  factory.registerFromPlugin(loader.getOSName("asr_OfferSeat_bt_node"));
  factory.registerFromPlugin(loader.getOSName("asr_SayInfo_bt_node"));



  auto blackboard = BT::Blackboard::create();

  std::string pkgpath = ros::package::getPath("speech");
  std::string xml_file = pkgpath + "/behavior_tree/speech_bt.xml";

  BT::Tree tree = factory.createTreeFromFile(xml_file, blackboard);
  auto publisher_zmq = std::make_shared<BT::PublisherZMQ>(tree, 10, 1666, 1667);

  ros::Rate loop_rate(10);

  int count = 0;

  bool finish = false;
  while (ros::ok() && !finish)
  {
    finish = tree.rootNode()->executeTick() == BT::NodeStatus::SUCCESS;

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}