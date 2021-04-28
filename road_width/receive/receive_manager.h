//
// Created by renjie on 2021/1/7.
//

#ifndef FOD_RECEIVE_MANAGER_H
#define FOD_RECEIVE_MANAGER_H



#include"baseReceiver.h"
#include"live_msg/liveMsgReceiver.h"
#include"topic/topicReceiver.h"

#include"common_include.h"

class ReceiveManager{
private:
    vector<BaseReceiver*> receivers_;
public:
    ReceiveManager()=default;
    void addReceiver(BaseReceiver* receiver);
    void start();
    void stop();
};

#endif //FOD_RECEIVE_MANAGER_H
