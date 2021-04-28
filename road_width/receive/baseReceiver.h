//
// Created by renjie on 2021/1/7.
//

#ifndef FOD_BASERECEIVE_H
#define FOD_BASERECEIVE_H
#include"container.h"
class BaseReceiver{
public:
    virtual void setDataContainerPtr( void* ptr) = 0;
    virtual void start() = 0;
    virtual void stop() = 0;
};

#endif //FOD_BASERECEIVE_H
