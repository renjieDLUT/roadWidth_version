//
// Created by renjie on 2020/11/27.
//

#ifndef ROAD_WIDTH_V2_ROAD_WIDTH_MANAGER_H
#define ROAD_WIDTH_V2_ROAD_WIDTH_MANAGER_H

#include "receiver_manager.h"
namespace ca{
    class Manager{
    public:
        Manager();
        ~Manager();
        void init();
        void initReceiver();
        void initProtobufContainer();
        void initRoadWidth();
        void start();
        void run();

    private:
        ReceiveManager receiveManager_;
        CRoadWidth* roadWidth;
        map< string, DataContainer< string>* > msgsMap_;

    };
}

#endif //ROAD_WIDTH_V2_ROAD_WIDTH_MANAGER_H
