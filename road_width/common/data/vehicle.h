//
// Created by renjie on 2020/11/27.
//

#ifndef ROAD_WIDTH_V2_VEHICLE_H
#define ROAD_WIDTH_V2_VEHICLE_H

namespace  ca{

    //车身参数
    struct Vehicle{
        double length;     //车长
        double width;      //车宽
        double minR;       //最小转弯半径
        double front;      //前悬
        double rear;       //后悬
    };

}


#endif //ROAD_WIDTH_V2_VEHICLE_H
