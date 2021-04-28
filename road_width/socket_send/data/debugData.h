//
// Created by renjie on 2021/3/16.
//

#ifndef ROADWIDTH_DEBUGDATA_H
#define ROADWIDTH_DEBUGDATA_H


#pragma pack(1)
struct DebugData1{
    unsigned char slotNum;
    unsigned char gridmap[40000];
    unsigned char width;
    unsigned char heigth;
    unsigned char resolution();
    int x;
    int y;
};

struct DebugData2{
    int roadWidth;
    int p0_x;
    int p0_y;
    int p1_x;
    int p1_y;
    int p2_x;
    int p2_y;
    int p3_x;
    int p3_y;
};

#endif //ROADWIDTH_DEBUGDATA_H
