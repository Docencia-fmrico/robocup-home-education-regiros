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

#ifndef RECEPCIONIST_PERSONINFO_H
#define RECEPCIONIST_PERSONINFO_H

#include "string"

namespace recepcionist {

    struct personInfo{
        std::string name;
        std::string drink;
        bool old; 
    };

    typedef struct personInfo PInfo;
}
#endif  // RECEPCIONIST_PERSONINFO_H