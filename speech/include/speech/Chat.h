
#include <gb_dialog/DialogInterface.h>
#include <string>
#include "std_msgs/String.h"

namespace ph = std::placeholders;

namespace speech
{
class Chat : public gb_dialog::DialogInterface
{
  public:
    Chat();

    void noIntentCB(dialogflow_ros_msgs::DialogflowResult result);

    void luggageIntentCB(dialogflow_ros_msgs::DialogflowResult result);

    void startCB(dialogflow_ros_msgs::DialogflowResult result);

    void askNameCB(dialogflow_ros_msgs::DialogflowResult result);

    void askAgeCB(dialogflow_ros_msgs::DialogflowResult result);

    void askDrinkCB(dialogflow_ros_msgs::DialogflowResult result);

  private:
    ros::NodeHandle nh_;
    ros::Publisher pub_param_;
};
};  // namespace find_my_mates
