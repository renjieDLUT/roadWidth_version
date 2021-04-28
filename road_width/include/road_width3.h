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
#include<Eigen/Dense>
#include"zros/ca_apa_node_transport/ca_apa_agent.pb.h"
#include"zros/core/shared_buffer.h"
#include"zros/common_transport/grid_map.pb.h"
#include"zros/core/transport.h"
#include<mutex>
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
    typedef zros::ca_apa_transport::apa_agent::ApaGlobalVehiclePose_T ApaGlobalVehiclePose_T;
    typedef zros::common_transport::grid_map::ExtendedOccupancyGrid ExtendedOccupancyGrid;

    class CRoadWidth{
    private:
        int _state;                                        //车辆状态,1: 确认车位中     2：泊车过程中
        Point3d _pose;                                  //车辆的位姿
        vector<Slot> _slots;                              //车位信息
        int _idTrackSlot;
        Slot _pointSelectSlot;                                 //选择车位
        int _flagSlotSide;                                    //1: 右侧     2：左侧

        ExtendedOccupancyGrid   _gridmap;    //订阅的gridmap
        ApaGlobalVehiclePose_T  _globalPose;
        zros::core::Subscriber::SharedPtr _gridmap_subscriber;
        zros::core::Subscriber::SharedPtr _globalPose_subscriber;
        zros::core::SharedBuffer::SharedPtr share_buffer_free;
        bool _flag1=false,_flag2=false;

        Mat _map;                                   //gridmap转成的图像
        array<double ,3> _parkCoord;            //泊车坐标系,0:x   1:y   2:theta
        array<double ,3> _mapCoord;            //泊车坐标系,0:x   1:y   2:theta
        vector<double> _roadWidth;
        double _dtc;

        double _scale;         //分辨率
        double _mFront;
        double _mRear;
        double _thresholdAngle;  //遍历旋转角度的左右阈值，默认值为10度
        int _NUM=3;
        double _maxRoadWidth;
        int _mode;    // 1:参考车辆的航向方向计算旋转基准    2：参考车位的P0，P1的方向
        Vehicle _veh;
        std::mutex m1_,m2_;

    public:
        //CRoadWidth(int state, ExtendedOccupancyGrid gridmap, Point2d pose, vector<Slot> slots,array<double ,3> parkCoord);
        CRoadWidth(int state, double scale=10, double mFront=300, double mRear=300, double thresholdAngle=0.02, double maxRoadWidth=700, int mode=1,Vehicle veh=Vehicle{450,180,500,100,100});

        CRoadWidth(int state,  Mat map, Point3d pose, vector<Slot> slots, array<double ,3> parkCoord, array<double ,3> mapCoord,
                   double scale=10, double mFront=300, double mRear=300, double thresholdAngle=0.02, double maxRoadWidth=700, int mode=1,Vehicle veh=Vehicle{450,180,500,100,100});


        CRoadWidth(int state, array<double ,3> parkCoord, array<double ,3> mapCoord, Slot pointSelectSlot,int flagSlotSide,
                   double scale=10, double mFront=300, double mRear=300, double thresholdAngle=0.02, double maxRoadWidth=700, int mode=1,Vehicle veh=Vehicle{450,180,500,100,100});

        void start();
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
        double getDTC();
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
        void roadWidth();
    };

}


#endif //ROAD_WIDTH_ROAD_WIDTH_H

