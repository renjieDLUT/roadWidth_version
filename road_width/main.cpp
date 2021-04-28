//
// Created by renjie on 2020/10/15.
//

//#include"road_width3.h"
#include<signal.h>
#include<chrono>
#include <thread>
#include"road_width4.h"
void signalHandler(int signum){
    cout<<"Interrupt signal ("<<signum<<") received"<<endl;
    exit(signum);
}

int main(int argc, char* argv[]){
    //signal(SIGINT,signalHandler);

#ifdef offline_ros
    ros::init(argc,argv,"roadWidth");
    ros::NodeHandle nh;
    ca::CRoadWidth roadWidth(nh);
#endif

#ifdef soc
    ca::CRoadWidth roadWidth;
#endif
    roadWidth.init();
    roadWidth.threadStart();

#ifdef soc
    while(true)
#endif
#ifdef offline_ros
    while(ros::ok())
#endif
    {
        vector<double> res=roadWidth.getRoadWidth();
        for(auto r:res){
            cout<<"***************roadwidth :"<<r<<"*******************"<<endl;
        }
        //cout<<"dtc :"<<  roadWidth.dtc(500,1,1)<<endl;
        this_thread::sleep_for(chrono::milliseconds(1000));

    }

#ifdef offline_ros
    ros::waitForShutdown();
#endif

//    ca::Manager manager;
//    manager.start();
}
