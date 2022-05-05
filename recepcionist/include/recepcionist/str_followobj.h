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

#ifndef RECEPCIONIST_STR_FOLLOWOBJ_H
#define RECEPCIONIST_STR_FOLLOWOBJ_H


namespace recepcionist {

    struct PointTF{
        double x;
        double y;
        double z;
        double dist;
        double or_x;
        double or_y;
        double or_z;
        double or_w;
    };
    
    typedef struct PointTF PointTF;

}
#endif  // RECEPCIONIST_STR_FOLLOWOBJ_H