#ifndef SPEECH_PERSONINFO_H
#define SPEECH_PERSONINFO_H

#include "string"

namespace speech {

    struct personInfo{
        std::string name;
        std::string drink;
        bool Old; 
    };

    typedef struct personInfo PInfo;
}
#endif  // SPEECH_PERSONINFO_H