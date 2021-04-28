//
// Created by renjie on 2020/11/30.
//

#include"road_width_manager.h"
namespace  ca{
    Manager::Manager(){
        init();
    }

    Manager::~Manager(){
        for(auto i:receiveManager_.getReceivers()){
            if(i== nullptr){
                delete i;
            }
        }
    }

   void  Manager::init(){
        initReceiver();
        initProtobufContainer();
        initRoadWidth();
    }

    void Manager::initReceiver(){
        BaseReceiver* base1=new TopicReceiver<zros::common_transport::grid_map::ExtendedOccupancyGrid>("fused_grid_map.topic");
        receiveManager_.addReceiver(base1);

        BaseReceiver* base2=new TopicReceiver<zros::ca_apa_transport::apa_agent::ApaGlobalVehiclePose_T>("ca_apa_car_pose.topic");
        receiveManager_.addReceiver(base2);

        receiveManager_.init();
    }

    void Manager::initProtobufContainer(){
        for(auto i:receiveManager_.getReceivers()){
            msgsMap_[i->getName()]=i->getstringMsg();
        }
    }

    void initRoadWidth(){
        roadWidth=new CRoadWidth();
        for(auto i:msgsMap_){
            roadWidth->addMsgContainer(i.first,i.second);
        }
    }

    void Manager::start(){
        receiveManager_.start();

        while(true){
            zros::ca_apa_transport::apa_agent::ApaGlobalVehiclePose_T pose;
            pose.ParseFromString(msgsMap_["ca_apa_car_pose.topic"]->getData());
            //pose= dynamic_cast<zros::ca_apa_transport::apa_agent::ApaGlobalVehiclePose_T*>(msgsMap_["ca_apa_car_pose.topic"]->getData());
            cout<<"x: "<<pose.globalvehx()<<"; y: "<<pose.globalvehy() <<"; angle: "<<pose.globalvehangle() <<endl;
            this_thread::sleep_for(std::chrono::milliseconds(500));
        }

    }


}

