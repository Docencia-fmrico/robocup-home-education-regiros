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

    void noIntentCB(dialogflow_ros_msgs::DialogflowResult result);

    void startCB(dialogflow_ros_msgs::DialogflowResult result);

    void askNameCB(dialogflow_ros_msgs::DialogflowResult result);

    bool done_;
    std::string param_;
    std::string response_;
  private:
    ros::NodeHandle nh_;
};
};  // namespace find_my_mate

#endif  // FIND_MY_MATE_CHAT_H