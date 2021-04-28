//
// Created by renjie on 2020/11/27.
//

#ifndef ROAD_WIDTH_V2_ROADWIDTH_PARAMETER_H
#define ROAD_WIDTH_V2_ROADWIDTH_PARAMETER_H
#include"container.h"
#include"common_include.h"

namespace ca{
    namespace roadWidth{
        struct Parameter{
            int function;    // 1. roadWidth     2. lane    3.all
            int debugBool;  //  1.true 0.false
            int debugMode;  // 0. screen  1. file
            int calMode;    //  1. call function    0.period
            double scale;         //分辨率
            double       front;         //计算通车道前向距离
            double        rear;         //计算通车道后向距离
            double   thresholdAngle;    //遍历旋转角度的左右阈值
            int           num;          //遍历范围内的离散个数
            double    maxRoadWidth;     //最大通车道宽度
            int           mode;         // 1:参考车辆的航向方向计算旋转基准    2：参考车位的P0，P1的方向
            double    freeThreshold;
            double    occupiedThreshold;
            int       modeMap;
            int   period;
        };

#ifdef soc
//        struct Input{
//            DataContainer<ApaGlobalVehiclePose_T> vehiclePose;
//            DataContainer<ExtendedOccupancyGrid> gridMap;
//        };
//
//        typedef DataContainer<Input> InputContainer;
#endif

        struct HoughLinesPPara{
            double rho;
            double theta;
            int threshold;
            int minLineLength ;
            int maxLineGap;
        };


    }
}


#endif //ROAD_WIDTH_V2_ROADWIDTH_PARAMETER_H
