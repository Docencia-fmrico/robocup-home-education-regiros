#include <gb_dialog/DialogInterface.h>
#include <string>

namespace ph = std::placeholders;

namespace gb_dialog
{
class Speech: public DialogInterface
{
  public:
    Speech(): nh_()
    {
      this->registerCallback(std::bind(&Speech::noIntentCB, this, ph::_1), "Default Fallback Intent");
      this->registerCallback(
        std::bind(&Speech::welcomeIntentCB, this, ph::_1),
        "Default Welcome Intent");
      this->registerCallback(std::bind(&Speech::luggageIntentCB, this, ph::_1), "Luggage Intent");
    }

    void noIntentCB(dialogflow_ros_msgs::DialogflowResult result)
    {
      Speech forwarder;
      ROS_INFO("[Speech] noIntentCB: intent [%s]", result.intent.c_str());
      speak(result.fulfillment_text);
      listen();
    }

    void welcomeIntentCB(dialogflow_ros_msgs::DialogflowResult result)
    {
      ROS_INFO("[Speech] welcomeIntentCB: intent [%s]", result.intent.c_str());
      speak(result.fulfillment_text);
    }

    void luggageIntentCB(dialogflow_ros_msgs::DialogflowResult result)
    {
      Speech forwarder;
      std::string color;

      for (const auto & param : result.parameters) {
          for (const auto & value : param.value) {
            color = value;
          }
      }

      ROS_INFO("luggageIntentCB: intent [%s]", result.intent.c_str());
      ROS_INFO("Color: %s", color.c_str());
      speak(result.fulfillment_text);
      if (color.empty()) {
        listen();
      }
      ros::shutdown();
    }
  
  bool done = false;

  private:
    ros::NodeHandle nh_;
};
}  // namespace gb_dialog

int main(int argc, char** argv)
{
  ros::init(argc, argv, "speech_node");
  gb_dialog::Speech forwarder;

  forwarder.speak("Avoid error");

  forwarder.speak("Which one is your luggage?");

  forwarder.listen();
  ros::spin();
  return 0;
}