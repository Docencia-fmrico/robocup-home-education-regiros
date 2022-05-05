#include <gb_dialog/DialogInterface.h>
#include <string>
#include "std_msgs/String.h"
#include "find_my_mate/Chat.h"

namespace ph = std::placeholders;

namespace find_my_mate
{

    Chat::Chat() 
    : nh_()
    {
      pub_param_ = nh_.advertise<std_msgs::String>("/speech/param", 1);
      this->registerCallback(std::bind(&Chat::noIntentCB, this, ph::_1), "Default Fallback Intent");
      this->registerCallback(std::bind(&Chat::luggageIntentCB, this, ph::_1), "Luggage Intent");
      this->registerCallback(std::bind(&Chat::startCB, this, ph::_1), "Start");
      this->registerCallback(std::bind(&Chat::askNameCB, this, ph::_1), "Ask Name");

    }

    void 
    Chat::noIntentCB(dialogflow_ros_msgs::DialogflowResult result)
    {
      ROS_INFO("[find_my_mate] noIntentCB: intent [%s]", result.intent.c_str());
      speak(result.fulfillment_text);
      listen();
    }

    void 
    Chat::startCB(dialogflow_ros_msgs::DialogflowResult result)
    {
      std_msgs::String start;

      start.data = "start";

      ROS_INFO("startCB: intent [%s]", result.intent.c_str());
 
      speak(result.fulfillment_text);
      if (start.data.empty()) {
        listen();
      }
      else {
        pub_param_.publish(start);
      }
    }

    void 
    Chat::askNameCB(dialogflow_ros_msgs::DialogflowResult result)
    {
      std_msgs::String name;

      for (const auto & param : result.parameters) {
        for (const auto & value : param.value) {
          name.data = value;
        }
      }

      ROS_INFO("askNameCB: intent [%s]", result.intent.c_str());
 
      speak(result.fulfillment_text);
      if (name.data.empty()) {
        listen();
      }
      else {
        pub_param_.publish(name);
      }
    }

    void 
    Chat::luggageIntentCB(dialogflow_ros_msgs::DialogflowResult result)
    {
      std_msgs::String color;

      for (const auto & param : result.parameters) {
        for (const auto & value : param.value) {
          color.data = value;
        }
      }

      ROS_INFO("luggageIntentCB: intent [%s]", result.intent.c_str());
      ROS_INFO("Name: %s", color.data.c_str());
      speak(result.fulfillment_text);
      if (color.data.empty()) {
        listen();
      }
      else {
        pub_param_.publish(color);
      }
    }

};  // namespace find_my_mate
