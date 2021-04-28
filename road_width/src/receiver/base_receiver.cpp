//
// Created by renjie on 2020/11/27.
//

#include "base_receiver.h"

namespace ca{
    BaseReceiver::BaseReceiver(string name){
        name_=name;

    }
    string BaseReceiver::getName(){
        return name_;
    }
    DataContainer<string>* BaseReceiver::getstringMsg(){
        return stringMsg_;
    }
//    DataPtrContainer<google::protobuf::Message*>* BaseReceiver::getMsgBasePtrContainer(){
//        return  msgBasePtrContainer_;
//    }
}