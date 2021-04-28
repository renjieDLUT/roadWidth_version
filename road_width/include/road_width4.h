//
// Created by renjie on 2020/11/27.
//

#ifndef ROAD_WIDTH_V2_ROAD_WIDTH4_H
#define ROAD_WIDTH_V2_ROAD_WIDTH4_H

#include "common_include.h"
#include"config.h"
#include"container.h"
#include"roadWidth_parameter.h"
#include"slot.h"
#include"vehicle.h"
#include"geometryMsgs.h"
#ifdef soc
#include"receive/receive_manager.h"
#define FILE  "/zros/res/capa/cfg.yaml"
#endif

#ifdef offline_ros
#define FILE  "/home/renjie/桌面/APA6/ZROS_SDK/20210303/SDK_A_CA_S202_dev_20210303110408/projects/product/capa_project/workspace/src/road_width_v2/cfg.yaml"
#endif
#define PI 3.1415926



namespace  ca{
    class CRoadWidth{
    private:
        int state_;

#ifdef soc
        DataContainer<ExtendedOccupancyGrid> gridmapContainer_;
        //DataContainer<ApaGlobalVehiclePose_T> globalPoseContainer_;
        DataContainer<CAPA_PP_DR>* DRContainer_;
        DataContainer<ApaSlotsInfo_T> slotsContainer_;

        zros::core::Subscriber::SharedPtr gridmap_subscriber_ ,   slot_subscriber_;
       // zros::core::Subscriber::SharedPtr globalPose_subscriber_;
        zros::core::SharedBuffer::SharedPtr share_buffer_free_ , share_buffer_occupied_;

        ReceiveManager* receiveManager_;
#endif

#ifdef offline_ros
        ros::NodeHandle nh_;
        ros::Subscriber subGridMap_, subDR_, subSlot_;
        ros::SubscribeOptions opsGridMap_,opsDR_, opsSlot_;
        ros::CallbackQueue gridMapQueue_, odometryQueue_, slotQueue_;
        ros::AsyncSpinner spinnerGridMap_, spinnerDR_, spinnerSlot_;
        DataContainer<nav_msgs::OccupancyGrid> occupancyGridContainer_;
        DataContainer<nav_msgs::Odometry> odometryContainer_;
        DataContainer<visualization_msgs::MarkerArray> slotsContainer_;
        ros::Publisher pubLine_, pubRW_, pubPoint_;
#endif
        roadWidth::Parameter paramRW_;
        Vehicle paramVeh_;
        roadWidth::HoughLinesPPara houghPara;

        Mat map_;                                   //gridmap转成的图像
        array<double ,3> parkCoord_;            //泊车坐标系,0:x   1:y   2:theta
        array<double ,3> mapCoord_;            //gridmap坐标系,0:x   1:y   2:theta
        Point3d pose_;                                  //车辆的位姿
        Slots pldSlots_;
        Slot slotSelected_;                             //选择的车位

        thread job_;
        bool threadSwitch_;                       //true:线程工作    false:线程停止
        DataContainer<vector<double>> roadWidth_;
        DataContainer<double>     dtc_;

    private:
        void initParam();
        void initSubscriber();
        void startSubscriber();

        void doJob();
        void roadWidth();
        bool getMsg();
        void toMat();
        void findBaseAngle(vector<double>& baseAngle);
        void linspace(double start,double end,int num,vector<double>& result);
        void calRoadWidth_1(vector<double>& angles, vector<vector<double>>& setRoadWidth);
        void Rotate(const Mat &srcImage, Mat &destImage, double angle);
        void findReferencePixel(const double& angle,vector<CvPoint>& pixels);
        void point2D2pixel(const Point2d& point2D,const double& angle, CvPoint &pixel);
        void pixel2world(const CvPoint& pixel, array<double, 2>& worldPose);
        void world2veh(const array<double,2>& worldPose, array<double,2>& mapPose);
        CvPoint getPointAffinedPos(const CvPoint &src, const CvPoint &center, double angle);
        double roadWidthNoTraverse(Mat& mat,CvPoint& pixel,int left,int right,const int number,const double angle, int& top, int& bottom);
        bool isFreeRegion(Mat& mat);
        bool validInclude(const int& top, const int& bottom, int left,int right, const CvPoint& p1, const CvPoint& p2);

        double dtc(double R,int side, int gear); //左正  右负;gear=-1,后退  ，gear=1,前进
        void lane();


    public:
#ifdef offline_ros
        CRoadWidth(ros::NodeHandle nh);
        void subGridMapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg);
        void subDRCallback(const nav_msgs::Odometry::ConstPtr &msg);
        void subSlotCallback(const visualization_msgs::MarkerArray::ConstPtr &msg);
        void publishLane(const vector< array< array<double , 2> , 2> >& points);
        void publishRW(const vector<int>& left, const vector<int>& right, const vector<int>& top, const vector<int>& bottom, const double& angle);
        void publishKeyPoint(Point2d point);
#endif

#ifdef soc
	CRoadWidth();
	 bool getSlots(ApaSlotsInfo_T& slots);
	 void addRoadWidthForSlot(const Bus_SlotOrigrn &src, Bus_Slots &res);
	 void addRoadWidthForSlot(const Bus_SlotOrigrn &src, Geometry::Pose2D &pose,  Bus_Slots &res);
	// bool getGlobalPose(ApaGlobalVehiclePose_T& dr);

#endif
        void init();
        void threadStart();
        void stop();
        vector<double> getRoadWidth();
        double getDTC();

#ifdef soc
        ApaSlotsInfo_T getSlots();
       // ApaGlobalVehiclePose_T getGlobalVehiclePose();
#endif


    };

}


#endif //ROAD_WIDTH_V2_ROAD_WIDTH4_H
