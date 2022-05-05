// Copyright 2022 Regiros
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <gb_dialog/DialogInterface.h>
#include <string>
#include "std_msgs/String.h"
#include "recepcionist/Chat.h"

namespace ph = std::placeholders;

namespace recepcionist
{
  Chat::Chat()
  : nh_(),
  done_(false),
  param_("null")
  {
    pub_param_ = nh_.advertise<std_msgs::String>("/speech/param", 1);
    this->registerCallback(std::bind(&Chat::noIntentCB, this, ph::_1), "Default Fallback Intent");
    this->registerCallback(std::bind(&Chat::luggageIntentCB, this, ph::_1), "Luggage Intent");
    this->registerCallback(std::bind(&Chat::startCB, this, ph::_1), "Start");
    this->registerCallback(std::bind(&Chat::askNameCB, this, ph::_1), "Ask Name");
    this->registerCallback(std::bind(&Chat::askDrinkCB, this, ph::_1), "Ask Drink");
    this->registerCallback(std::bind(&Chat::askAgeCB, this, ph::_1), "Ask Age");
  }

  void
  Chat::noIntentCB(dialogflow_ros_msgs::DialogflowResult result)
  {
    ROS_INFO("[Speech] noIntentCB: intent [%s]", result.intent.c_str());
    speak(result.fulfillment_text);
    listen();
  }

  void
  Chat::luggageIntentCB(dialogflow_ros_msgs::DialogflowResult result)
  {
    std::string color;

    for (const auto & param : result.parameters)
    {
      for (const auto & value : param.value)
      {
        color = value;
      }
    }

    ROS_INFO("luggageIntentCB: intent [%s]", result.intent.c_str());
    ROS_INFO("Color: %s", color.c_str());
    speak(result.fulfillment_text);
    if (color.empty())
    {
      listen();
    }
    else
    {
      this->param_ = color;
      this->done_ = true;
    }
  }

  void
  Chat::startCB(dialogflow_ros_msgs::DialogflowResult result)
  {
    std_msgs::String start;

    if (done_) {
      return;
    }
    start.data = "start";

    ROS_INFO("startCB: intent [%s]", result.intent.c_str());

    this->done_ = true;
    this->response_ = result.fulfillment_text;
  }

  void
  Chat::askNameCB(dialogflow_ros_msgs::DialogflowResult result)
  {
    std::string name;

    for (const auto & param : result.parameters)
    {
      for (const auto & value : param.value)
      {
        name = value;
      }
    }

    ROS_INFO("askNameCB: intent [%s]", result.intent.c_str());

    this->response_ = result.fulfillment_text;
    if (name.empty())
    {
      listen();
    }
    else
    {
      this->param_ = name;
      this->done_ = true;
    }
  }

  void
  Chat::askAgeCB(dialogflow_ros_msgs::DialogflowResult result)
  {
    std::string age;

    for (const auto & param : result.parameters)
    {
      for (const auto & value : param.value)
      {
        age = value;
      }
    }

    ROS_INFO("askAgeCB: intent [%s]", result.intent.c_str());

    this->response_ = result.fulfillment_text;
    if (age.empty())
    {
      listen();
    }
    else
    {
      this->param_ = age;
      this->done_ = true;
    }
  }

  void
  Chat::askDrinkCB(dialogflow_ros_msgs::DialogflowResult result)
  {
    std::string drink;

    for (const auto & param : result.parameters)
    {
      for (const auto & value : param.value)
      {
        drink = value;
      }
    }

    ROS_INFO("askDrinkCB: intent [%s]", result.intent.c_str());

    this->response_ = result.fulfillment_text;
    if (drink.empty())
    {
      listen();
    }
    else
    {
      this->param_ = drink;
      this->done_ = true;
    }
  }
};  // namespace recepcionist

