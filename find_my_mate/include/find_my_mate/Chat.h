#ifndef FIND_MY_MATE_CHAT_H
#define FIND_MY_MATE_CHAT_H

#include <gb_dialog/DialogInterface.h>
#include <string>
#include "std_msgs/String.h"



namespace ph = std::placeholders;

namespace find_my_mate
{
class Chat : public gb_dialog::DialogInterface
{
  public:
    Chat();

    void 
    noIntentCB(dialogflow_ros_msgs::DialogflowResult result);

    void 
    startCB(dialogflow_ros_msgs::DialogflowResult result);

    void 
    askNameCB(dialogflow_ros_msgs::DialogflowResult result);

    void 
    luggageIntentCB(dialogflow_ros_msgs::DialogflowResult result);

  private:
    ros::NodeHandle nh_;
    ros::Publisher pub_param_;
};
}  // namespace find_my_mate
#endif  // FIND_MY_MATE_CHAT_H