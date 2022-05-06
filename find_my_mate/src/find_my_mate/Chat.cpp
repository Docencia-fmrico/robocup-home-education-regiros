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
#include "find_my_mate/Chat.h"

namespace ph = std::placeholders;

namespace find_my_mate
{
  Chat::Chat()
  : nh_(),
  done_(false),
  param_("null")
  {
    this->registerCallback(std::bind(&Chat::noIntentCB, this, ph::_1), "Default Fallback Intent");
    this->registerCallback(std::bind(&Chat::startCB, this, ph::_1), "Start");
    this->registerCallback(std::bind(&Chat::askNameCB, this, ph::_1), "Ask Name");
  }

  void
  Chat::noIntentCB(dialogflow_ros_msgs::DialogflowResult result)
  {
    ROS_INFO("[Speech] noIntentCB: intent [%s]", result.intent.c_str());
    speak(result.fulfillment_text);
    listen();
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
};  // namespace find_my_mate