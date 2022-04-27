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

#ifndef ROBOCUP_HOME_EDUCATION_STR_FOLLOWOBJ_H
#define ROBOCUP_HOME_EDUCATION_STR_FOLLOWOBJ_H


namespace robocup_home_education {

    struct objectinimage{
        bool detected;
        int x;
        int y;
        float depth; 
    };
    
    typedef struct objectinimage objectinimage;

    struct speeds{
        double angular;
        double linear;
    };

    typedef struct speeds speeds;
}
#endif  // ROBOCUP_HOME_EDUCATION_STR_FOLLOWOBJ_H