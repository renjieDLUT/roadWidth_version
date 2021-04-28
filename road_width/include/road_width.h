//
// Created by renjie on 2020/10/15.
//

#ifndef ROAD_WIDTH_ROAD_WIDTH_H
#define ROAD_WIDTH_ROAD_WIDTH_H


#include<iostream>
#include <vector>
#include<array>
#include<opencv2/core/core.hpp>
#include<opencv2/opencv.hpp>
#include<eigen3/Eigen/Core>
//#include<zros/ca_remote_proto/grid_map.pb.h>
#define PI 3.1415926
using namespace std;
using namespace cv;
using namespace Eigen;

namespace ca{

struct Vehicle{
    double length;
    double width;
    double minR;
    double front;
    double rear;
};

typedef vector<Point2d> Slot;
//typedef zros::common_transport::ca_grid_map::ExtendedOccupancyGrid ExtendedOccupancyGrid;

class CRoadWidth{
private:
    int _state;                                        //车辆状态,1: 确认车位中     2：泊车过程中
    Point3d _pose;                                  //车辆的位姿
    vector<Slot> _slots;                              //车位信息
    int _idTrackSlot;
    Slot _pointSelectSlot;                                 //选择车位
    int _flagSlotSide;                                    //1: 右侧     2：左侧

    //ExtendedOccupancyGrid   _gridmap;    //订阅的gridmap
    Mat _map;                                   //gridmap转成的图像
    array<double ,3> _parkCoord;            //泊车坐标系,0:x   1:y   2:theta
    array<double ,3> _mapCoord;            //泊车坐标系,0:x   1:y   2:theta
    vector<double> _roadWidth;

    double _scale;         //分辨率
    double _mFront;
    double _mRear;
    double _thresholdAngle;  //遍历旋转角度的左右阈值，默认值为10度
    int _NUM=3;
    double _maxRoadWidth;
    int _mode;    // 1:参考车辆的航向方向计算旋转基准    2：参考车位的P0，P1的方向
    Vehicle _veh;

public:
    //CRoadWidth(int state, ExtendedOccupancyGrid gridmap, Point2d pose, vector<Slot> slots,array<double ,3> parkCoord);
    CRoadWidth(int state,  Mat map, Point3d pose, vector<Slot> slots, array<double ,3> parkCoord, array<double ,3> mapCoord,
               double scale=10, double mFront=500, double mRear=500, double thresholdAngle=10, double maxRoadWidth=700, int mode=1,Vehicle veh=Vehicle{450,180,500});
    CRoadWidth(int state, array<double ,3> parkCoord, array<double ,3> mapCoord, Slot pointSelectSlot,int flagSlotSide,
               double scale=10, double mFront=500, double mRear=500, double thresholdAngle=10, double maxRoadWidth=700, int mode=1,Vehicle veh=Vehicle{450,180,500});

    void setState(int state);
    //void setGridmap(ExtendedOccupancyGrid gridmap);
    void setMap(Mat map);
    void setPose(Point3d pose);
    void setSlots(vector<Slot> slots);
    void setParkCoord(array<double ,3> parkCoord);
    void setMapCoord(array<double ,3> mapCoord);
    void setIDTrackSlot(int idTrackSlot);
    void setSelectSlot(Slot pointSelectSlot);
    vector<double> getRoadWidth();
    vector<double> roadWidth();
    double dtc(double R,int side, int gear); //左正  右负;gear=-1,后退  ，gear=1,前进

private:
    void toMat();
    void point2D2pixel(Point2d point2D,double angle, CvPoint &pixel);
    void findReferencePixel(double angle,vector<CvPoint>& pixels);
    double roadWidthNoTraverse(Mat& mat,CvPoint& pixel,int left,int right);
    vector<double> linspace(double start,double end,int num);
    void Rotate(const Mat &srcImage, Mat &destImage, double angle);
    CvPoint getPointAffinedPos(const CvPoint &src, const CvPoint &center, double angle);
    bool isFreeRegion(Mat mat);
    void positionW(Point3d pose, vector<Point2d>& pVeh);

};

}


#endif //ROAD_WIDTH_ROAD_WIDTH_H
