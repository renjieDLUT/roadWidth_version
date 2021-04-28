//
// Created by renjie on 2020/11/27.
//
#include"receiver_manager.h"
namespace ca{
    void ReceiveManager::addReceiver(BaseReceiver* receiver){
        receivers_.push_back(receiver);
    }

    vector<BaseReceiver*> ReceiveManager::getReceivers(){
        return receivers_;
    }

    void ReceiveManager::init(){
        for(auto i:receivers_){
            i->init();
        }
    }

    void ReceiveManager::start(){
        if(receivers_.empty()){
            cerr<<"error:************don't add any receiver*************"<<endl;
            return;
        }
        for(auto i:receivers_){
            i->start();
        }
    }

    void ReceiveManager::stop(){
        for(auto i:receivers_){
            i->stop();
        }
    }
}
