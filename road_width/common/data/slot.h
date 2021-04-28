//
// Created by renjie on 2020/11/27.
//

#ifndef ROAD_WIDTH_V2_SLOT_H
#define ROAD_WIDTH_V2_SLOT_H
typedef int int32_T;
namespace ca{

    struct Point{
        bool flag;   //false:无效  true:有效
        double x;
        double y;
    };

    struct Slot{
        int slotSide;  //1: 右侧     2：左侧
        Point p0;
        Point p1;
        Point p2;
        Point p3;
    };

    struct Slots{
        int num;
        vector<Slot> slots;
    };

    typedef struct {
        int32_T SlotID;
        int32_T P0X;
        int32_T P0Y;
        int32_T P0Prop;
        int32_T P1X;
        int32_T P1Y;
        int32_T P1Prop;
        int32_T P2X;
        int32_T P2Y;
        int32_T P2Prop;
        int32_T P3X;
        int32_T P3Y;
        int32_T P3Prop;
        int32_T P4X;
        int32_T P4Y;
        int32_T P5X;
        int32_T P5Y;
        int32_T EdgeType;
        int32_T Obj1Type;
        int32_T Obj2Type;
    } Bus_SlotOrigrn;

    typedef struct {
        Bus_SlotOrigrn origin;
        int32_T RoadWidth;
    } Bus_Slots;
}


#endif //ROAD_WIDTH_V2_SLOT_H
