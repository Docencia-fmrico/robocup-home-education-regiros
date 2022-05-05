

#ifndef FIND_MY_MATE_STR_INFO_H
#define FIND_MY_MATE_STR_INFO_H

#include "string"

namespace find_my_mate {

    struct Info_of_person{
        std::string color;
        std::string name;
        std::string object; 
    };

    typedef struct Info_of_person Infop;
}
#endif  // FIND_MY_MATE_STR_INFO_H