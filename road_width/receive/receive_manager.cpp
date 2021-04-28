//
// Created by renjie on 2021/1/7.
//

#include "receive_manager.h"

void ReceiveManager::addReceiver(BaseReceiver* receiver){
    receivers_.push_back(receiver);
}

void ReceiveManager::start() {
    for(auto receiver:receivers_){
        receiver->start();
    }
}

void ReceiveManager::stop() {
    for(auto receiver:receivers_){
        receiver->stop();
    }
}