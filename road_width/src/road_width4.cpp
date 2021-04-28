//
// Created by renjie on 2020/12/1.
//

#include"road_width4.h"



namespace  ca{
#ifdef offline_ros
    CRoadWidth::CRoadWidth(ros::NodeHandle nh):nh_(nh),spinnerGridMap_(ros::AsyncSpinner(0, &gridMapQueue_)), spinnerDR_(ros::AsyncSpinner(0, &odometryQueue_)), spinnerSlot_(ros::AsyncSpinner(0, &slotQueue_)){
        Config::setParameterFile(FILE);
        state_=1;
        pubLine_ = nh_.advertise<visualization_msgs::MarkerArray>("visualization_markerArray", 10);
        pubRW_ = nh_.advertise<visualization_msgs::MarkerArray>("roadWidth",10);
        pubPoint_=nh_.advertise<visualization_msgs::Marker>("KeyPoint",10);
    }
#endif

#ifdef soc
	CRoadWidth::CRoadWidth(){
        Config::setParameterFile(FILE);
        state_=1;
    }
#endif
    
    void CRoadWidth::init(){
        initParam();
        initSubscriber();
#ifdef soc
        if(paramRW_.calMode==1){
            DRContainer_= new DataContainer<CAPA_PP_DR>();
            receiveManager_=new ReceiveManager();
            BaseReceiver* baseReceiverPtr;
            baseReceiverPtr = new LiveMsgReceiver<CAPA_PP_DR>(LIVE_MSG_CATEGORY_CAPA_2CA, LIVE_MSG_OEM_ID(CAPA_PP_DR));
            baseReceiverPtr->setDataContainerPtr( DRContainer_ );
            receiveManager_->addReceiver( baseReceiverPtr );
        }
#endif
    }
    
    void CRoadWidth::initParam(){

        paramRW_.function=Config::get<int>("function");
        paramRW_.debugBool=Config::get<int>("debugBool");
        paramRW_.debugMode=Config::get<int>("debugMode");
        paramRW_.calMode=Config::get<int>("calMode");
        paramRW_.scale = Config::get<double>("scale");
        paramRW_.front = Config::get<double>("front");
        paramRW_.rear = Config::get<double>("rear");
        paramRW_.thresholdAngle = Config::get<double>("thresholdAngle");
        paramRW_.num = Config::get<int>("num");
        paramRW_.maxRoadWidth = Config::get< double>("maxRoadWidth");
        paramRW_.mode = Config::get<int>("mode");
        paramRW_.freeThreshold = Config::get<double>("freeThreshold");
        paramRW_.occupiedThreshold = Config::get<double>("occupiedThreshold");
        paramRW_.modeMap = Config::get<double>("modeMap");
        paramRW_.period = Config::get<int>("period");

        paramVeh_.length=Config::get<double>("vehicle_length");
        paramVeh_.width=Config::get<double>("vehicle_width");
        paramVeh_.minR=Config::get<double>("vehicle_minR");
        paramVeh_.front=Config::get<double>("vehicle_front");
        paramVeh_.rear=Config::get<double>("vehicle_rear");

        houghPara.rho=Config::get<double>("hough_rho");
        houghPara.theta=Config::get<double>("hough_theta_deg")/57.3;
        houghPara.threshold=Config::get<int>("hough_threshold");
        houghPara.minLineLength=Config::get<int>("hough_minLineLength");
        houghPara.maxLineGap=Config::get<int>("hough_maxLineGap");



#ifdef soc
        if(paramRW_.debugMode==1){
            string s=Config::get<string>("coutFile");
            freopen(s.c_str(),"w",stdout);
        }

#endif

    }

    void CRoadWidth::initSubscriber(){
#ifdef soc
        gridmap_subscriber_=zros::core::Transport::Instance()->CreateSubscriber("fused_grid_map.topic");

        if(paramRW_.calMode==1){
                    //globalPose_subscriber_=zros::core::Transport::Instance()->CreateSubscriber("ca_apa_car_pose.topic");
                    slot_subscriber_=zros::core::Transport::Instance()->CreateSubscriber("ca_apa_slot.topic");

        }
        //TrackedSlot_subscriber=zros::core::Transport::Instance()->CreateSubscriber("ca_apa_track_slot.topic");

        gridmap_subscriber_->RegisterCallback<ExtendedOccupancyGrid>(
                [this](const std::string& topic,const  ExtendedOccupancyGrid& msg )
                {
                    gridmapContainer_.upDate(msg);
                    //cout<<"receive gridmap message"<<endl;
                }
        );

        if(paramRW_.calMode==1){
//            globalPose_subscriber_->RegisterCallback<ApaGlobalVehiclePose_T>(
//                [this](const std::string& topic,const ApaGlobalVehiclePose_T& msg){
//                    globalPoseContainer_.upDate(msg);
//                    //cout<<"receive globalPose message"<<endl;
//                }
//            );
            slot_subscriber_->RegisterCallback<ApaSlotsInfo_T>(
                    [this](const std::string& topic,const  ApaSlotsInfo_T& msg ){
                        slotsContainer_.upDate(msg);
                    }
            );
        }

#endif

#ifdef offline_ros
        opsGridMap_ = getSubscribeOptions("/zros_dbg_node/local_map", 1, &CRoadWidth::subGridMapCallback, this, &gridMapQueue_);
        opsDR_ = getSubscribeOptions("/zros_dbg_node/dr_odometry", 1, &CRoadWidth::subDRCallback, this, &odometryQueue_);
        opsSlot_=getSubscribeOptions("/zros_dbg_node/slot_filter_markerarray", 1, &CRoadWidth::subSlotCallback, this, &slotQueue_);
        subGridMap_ = nh_.subscribe(opsGridMap_);
        subDR_=nh_.subscribe(opsDR_);
        subSlot_=nh_.subscribe(opsSlot_);
        //spinnerGridMap_=ros::AsyncSpinner(0, &gridMapQueue_);
        //spinnerDR_=ros::AsyncSpinner(0, &odometryQueue_);
#endif
    }
    
    void CRoadWidth::threadStart(){
        threadSwitch_= true;
        startSubscriber();
#ifdef soc
        if(paramRW_.calMode==1) receiveManager_->start();
#endif
        job_ = std::thread(std::bind(&CRoadWidth::doJob, this));
        job_.detach();
    }
    
    void CRoadWidth::startSubscriber(){
#ifdef soc
        gridmap_subscriber_->Start();
        if(paramRW_.calMode==1){
            //globalPose_subscriber_->Start();
            slot_subscriber_->Start();
        }

#endif

#ifdef offline_ros
        spinnerGridMap_.start();
        spinnerDR_.start();
        spinnerSlot_.start();
#endif
    }

#ifdef offline_ros
    void CRoadWidth::subGridMapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg){
        occupancyGridContainer_.upDate(*msg);
    }

    void CRoadWidth::subDRCallback(const nav_msgs::Odometry::ConstPtr &msg){
        odometryContainer_.upDate(*msg);
    }

    void CRoadWidth::subSlotCallback(const visualization_msgs::MarkerArray::ConstPtr &msg){
        slotsContainer_.upDate(*msg);
    }
#endif

    vector<double> CRoadWidth::getRoadWidth(){
        return roadWidth_.getData();
    }

    double CRoadWidth::getDTC(){
        return dtc_.getData();
    }

#ifdef soc
    ApaSlotsInfo_T CRoadWidth::getSlots(){
        return  slotsContainer_.getData();
    }

    void CRoadWidth::addRoadWidthForSlot(const Bus_SlotOrigrn &src, Bus_Slots &res){
        Slot slot;

        res.origin=src;
        res.RoadWidth=300;
        pldSlots_.slots.clear();
        slot.p0.x = src.P0X ;   slot.p0.y=src.P0Y;
        slot.p1.x = src.P1X ;   slot.p1.y=src.P1Y;
        slot.p2.x = src.P2X ;   slot.p2.y=src.P2Y;
        slot.p3.x = src.P3X ;   slot.p3.y=src.P3Y;

        if(slot.p1.y>slot.p3.y)        //bug
            slot.slotSide=1;
        else
            slot.slotSide=2;

        pldSlots_.slots.push_back(slot);
        roadWidth();
        auto roadwidth=getRoadWidth();
        res.RoadWidth=roadwidth[0];

    }

//    ApaGlobalVehiclePose_T CRoadWidth::getGlobalVehiclePose(){
//        return globalPoseContainer_.getData();
//    }

    void CRoadWidth::addRoadWidthForSlot(const Bus_SlotOrigrn &src, Geometry::Pose2D &pose,  Bus_Slots &res){
        Slot slot;
        res.origin=src;
        res.RoadWidth=300;
        pldSlots_.slots.clear();
        slot.p0.x = src.P0X ;   slot.p0.y=src.P0Y;
        slot.p1.x = src.P1X ;   slot.p1.y=src.P1Y;
        slot.p2.x = src.P2X ;   slot.p2.y=src.P2Y;
        slot.p3.x = src.P3X ;   slot.p3.y=src.P3Y;

        if(slot.p1.y>slot.p3.y)        //bug
            slot.slotSide=1;
        else
            slot.slotSide=2;

        pldSlots_.slots.push_back(slot);

        pose_.x=(double)pose.x;
        pose_.y=(double)pose.y;
        pose_.z=(double)pose.yaw/180*PI;

        auto t1=gridmapContainer_.getTimePoint();
        auto t2=chrono::system_clock::now();
        if(chrono::duration_cast<chrono::milliseconds>(t2-t1).count()>400) return;
        roadWidth();
        auto roadwidth=getRoadWidth();
        res.RoadWidth=roadwidth[0];
    }

#endif
    
    void CRoadWidth::doJob(){
        while(threadSwitch_){
            this_thread::sleep_for(std::chrono::milliseconds(paramRW_.period));
            if(!getMsg()) continue ;
            chrono::system_clock::time_point t1=chrono::system_clock::now();

            if(paramRW_.calMode==1)
                if(paramRW_.function==1||paramRW_.function==3) roadWidth();

            if(paramRW_.function==2||paramRW_.function==3) lane();


            auto t2=chrono::system_clock::now();
            cout<<"last time: "<<chrono::duration_cast<chrono::microseconds>(t2-t1).count()<<endl;
        }
    }
    
    void CRoadWidth::stop(){
        threadSwitch_= false;
#ifdef soc
        if(gridmap_subscriber_.get()) gridmap_subscriber_->Stop();
       // if(globalPose_subscriber_.get()) globalPose_subscriber_->Stop();
        if(slot_subscriber_.get())  slot_subscriber_->Stop();
        if(paramRW_.calMode==1) receiveManager_->stop();
#endif


    }
    
    bool CRoadWidth::getMsg(){
#ifdef soc
        CAPA_PP_DR globalPose;
        ApaSlotsInfo_T slots;

        if(paramRW_.calMode==1){
            if(gridmapContainer_.isNew()==false&&DRContainer_->isNew()==false)  return false;
            if(gridmapContainer_.getID()<2||DRContainer_->getID()<2) return false;
//            if(gridmapContainer_.isNew()==false||gridmapContainer_.getID()<2)  return false;

        }
        else{
            if(gridmapContainer_.isNew()==false||gridmapContainer_.getID()<2) return false;
        }

        if(gridmapContainer_.isNew()==true){
            //gridmap=gridmapContainer_.getData();
            toMat();
        }

        if(paramRW_.calMode==1){
            if(DRContainer_->isNew()==true){
                globalPose=DRContainer_->getData();
                pose_.x=(double)globalPose.OutDRX_cm;
                pose_.y=(double)globalPose.OutDRY_cm;
                pose_.z=(double)globalPose.OutDRAngle_deg/180*PI;
            }
            if(slotsContainer_.isNew()==true){
                pldSlots_.slots.clear();
                slots=slotsContainer_.getData();
                pldSlots_.num=slots.pldslot_size();
                cout<<"slots.pldslot_size():  "<<slots.pldslot_size()<<endl;
                for(int i=0;i<slots.pldslot_size();i++){
                    Slot slot;
                    slot.p0.x=slots.mutable_pldslot(i)->p0x();
                    slot.p0.y=slots.mutable_pldslot(i)->p0y();
                    slot.p1.x=slots.mutable_pldslot(i)->p1x();
                    slot.p1.y=slots.mutable_pldslot(i)->p1y();
                    slot.p2.x=slots.mutable_pldslot(i)->p2x();
                    slot.p2.y=slots.mutable_pldslot(i)->p2y();
                    slot.p3.x=slots.mutable_pldslot(i)->p3x();
                    slot.p3.y=slots.mutable_pldslot(i)->p3y();
                    if(slot.p1.y>slot.p3.y)        //bug
                        slot.slotSide=1;
                    else
                        slot.slotSide=2;

                    pldSlots_.slots.push_back(slot);
                }

            }
        }

        return true;
#endif

#ifdef offline_ros
        nav_msgs::Odometry globalPose;
        visualization_msgs::MarkerArray slots;
        if(occupancyGridContainer_.isNew()==false&&odometryContainer_.isNew()==false) return false;
        if(occupancyGridContainer_.getID()<2||odometryContainer_.getID()<2) return false;

        if(occupancyGridContainer_.isNew()==true){
            toMat();
        }

        if(odometryContainer_.isNew()==true){
            globalPose=odometryContainer_.getData();
            pose_.x=(double)globalPose.pose.pose.position.x*100;
            pose_.y=(double)globalPose.pose.pose.position.y*100;
            //tf::Vector3 v;
            tf::Quaternion q;
            tf::Matrix3x3 matrix;
            tf::quaternionMsgToTF(globalPose.pose.pose.orientation, q);
            matrix.setRotation(q);
            tfScalar yaw, pitch, roll;
            matrix.getEulerYPR(yaw,pitch,roll);
            pose_.z=(double)yaw;
            cout<<"*******************************pose_.z: "<<pose_.z<<endl;
        }

        if(slotsContainer_.isNew()==true){
            pldSlots_.slots.clear();
            slots=slotsContainer_.getData();
            for(auto slotMarker:slots.markers){
                if(slotMarker.id<100&&slotMarker.points.size()==8){
                    Slot slot;
                    slot.p1.x=slotMarker.points[0].x*100;
                    slot.p1.y=slotMarker.points[0].y*100;
                    slot.p3.x=slotMarker.points[1].x*100;
                    slot.p3.y=slotMarker.points[1].y*100;
                    slot.p2.x=slotMarker.points[3].x*100;
                    slot.p2.y=slotMarker.points[3].y*100;
                    slot.p0.x=slotMarker.points[5].x*100;
                    slot.p0.y=slotMarker.points[5].y*100;
                    pldSlots_.slots.push_back(slot);
                }
            }
        }
        return true;
#endif
    }
    
    void CRoadWidth::toMat(){
#ifdef soc
        ExtendedOccupancyGrid gridmap=gridmapContainer_.getData();
        int width=gridmap.info().width();
        int height=gridmap.info().height();
	//cout<<"*************function tomat  111111111111111**********"<<endl;
        std::vector<float> prob(width * height);
        map_.create(height , width , CV_8UC1);
	//cout<<"*************function tomat  222222222222222**********"<<endl;
        share_buffer_free_ = zros::core::SharedBuffer::ImportBuffer(gridmap.free_prob_tag());
        share_buffer_occupied_ = zros::core::SharedBuffer::ImportBuffer(gridmap.occupied_prob_tag());
        void* ptr_free = reinterpret_cast<void *>(share_buffer_free_->GetPointer());
        void* ptr_occupied = reinterpret_cast<void *>(share_buffer_occupied_->GetPointer());
        if(paramRW_.modeMap==1){
            memcpy(prob.data(), ptr_free, prob.size() * sizeof(float));
            for(int i = 0; i<height; i++){
                for(int j = 0; j<width; j++){
                    int tmp = i*width +j;
                    if(prob[tmp]>paramRW_.freeThreshold){
                        map_.at<uchar>(height-1-i,j)=255;
                    }
                    else{
                        map_.at<uchar>(height-1-i,j)=0;
                    }
                }
            }
        }
        else if(paramRW_.modeMap==2){
            memcpy(prob.data(), ptr_occupied, prob.size() * sizeof(float));
            for(int i = 0; i<height; i++){
                for(int j = 0; j<width; j++){
                    int tmp = i*width +j;
                    if(prob[tmp]>paramRW_.occupiedThreshold){
                        map_.at<uchar>(height-1-i,j)=0;
                    }
                    else{
                        map_.at<uchar>(height-1-i,j)=255;
                    }
                }
            }
        }
        
        mapCoord_[0]=gridmap.info().origin().position().x()*100;
        mapCoord_[1]=gridmap.info().origin().position().y()*100;
        mapCoord_[2]=0;
#endif

#ifdef offline_ros
        nav_msgs::OccupancyGrid  gridmap=occupancyGridContainer_.getData();
        map_.create(gridmap.info.height, gridmap.info.width, CV_8UC1);
        for(int i=0; i<gridmap.info.height; i++){
            for(int j=0;j<gridmap.info.width; j++){
                int tmp=i*gridmap.info.width+j;
                if(gridmap.data[tmp]>80)
                    map_.at<uchar>(gridmap.info.height-1-i,j)=0;
                else
                    map_.at<uchar>(gridmap.info.height-1-i,j)=255;
            }
        }
//        static int i=0;
//        i++;
//        if(i==10)
//            imwrite("/home/renjie/map.png",map_);
        mapCoord_[0]=gridmap.info.origin.position.x*100;
        mapCoord_[1]=gridmap.info.origin.position.y*100;
        mapCoord_[2]=0;
        Point2d p;
        p.x=mapCoord_[0];p.y=mapCoord_[1];
      //  publishKeyPoint(p);
#endif
    }

    void CRoadWidth::roadWidth(){
        vector<double> baseAngle;
        vector<double> angles;
        vector<vector<double>> setRoadWidth;
//        cout<<"********[roadWidth]********"<<endl;
        findBaseAngle(baseAngle);

        linspace(baseAngle[0] - paramRW_.thresholdAngle , baseAngle[0] + paramRW_.thresholdAngle , paramRW_.num , angles);
        calRoadWidth_1(angles,setRoadWidth);
        
        vector<double> roadWidth=setRoadWidth[0];
        for(int i=0;i<setRoadWidth.size();i++){
            for(int j=0;j<setRoadWidth[i].size();j++){
                if(roadWidth[j]<setRoadWidth[i][j]){
                    roadWidth[j]=setRoadWidth[i][j];
                }
            }
        }
        roadWidth_.upDate(roadWidth);

    }

    void CRoadWidth::findBaseAngle(vector<double>& baseAngle){

        if(state_==1){
            baseAngle.push_back(-pose_.z);
        }
//        else if(state_==1&&paramRW_.mode==2){
//            for(auto i:pldSlots_.slots){
//                double tmpX,tmpY,tmpAngle;
//                tmpX=i.p1.x-i.p0.x;
//                tmpY=i.p1.y-i.p0.y;
//                tmpAngle=atan2(tmpY,tmpX);
//                baseAngle.push_back(tmpAngle);
//            }
//        }
        else if(state_==2){
            baseAngle.push_back(parkCoord_[2]);
        }else{
            cout<<"[roadWidth] : no this case"<<endl;
            exit(0);
        }
    }

    void CRoadWidth::linspace(double start,double end,int num, vector<double>& result){
        if(num==1){
            result.push_back(start);
        }
        else if(num>1){
            double step=(end-start)/(num-1);
            for(int i=0;i<num;i++){
                result.push_back(start+step*i);
            }
        }
        else{
            cout<<"the value of num is error"<<endl;
            exit(0);
        }
    }
    
    void CRoadWidth::calRoadWidth_1 ( vector<double>& angles, vector<vector<double>>& setRoadWidth ){
        
        Mat destImage;
        vector<double> roadWidth;
        vector<CvPoint> referencePixel;
        
        for(auto angle:angles){
            roadWidth.clear();
            referencePixel.clear();
            Rotate(map_,destImage,angle);
            findReferencePixel(angle,referencePixel);
            vector<int> lefts,rights,tops,bottoms;
//            for(auto r:referencePixel){
//                circle(destImage,r,2,Scalar(0));
//            }
//            imwrite("/home/renjie/destImage.png",destImage);
            if(((state_==1&&paramRW_.mode==1)||state_==2)&&!(referencePixel.size()==1)){
                cout<<"[roadWidth] : state==1,mode==1 or state==2 ,referencePixel.size should be 1"<<endl;
                exit(2);
            }
            for(int i=0;i<referencePixel.size();i++){
                auto pixel=referencePixel[i];
                int left,right,top,bottom;
                left=min(max(pixel.x-(int)(paramRW_.front/paramRW_.scale),1), map_.cols-10);                         //避免left,right超出map_边界
                right=max( min(pixel.x+(int)(paramRW_.rear/paramRW_.scale), map_.cols), 10);
                double tmpRoadWidth=roadWidthNoTraverse(destImage,pixel,left,right,i,angle,top,bottom);
                roadWidth.push_back(tmpRoadWidth);
                lefts.push_back(left);
                rights.push_back(right);
                tops.push_back(top);
                bottoms.push_back(bottom);
            }
#ifdef offline_ros
            publishRW(lefts,rights,tops,bottoms,angle);
#endif
            setRoadWidth.push_back(roadWidth);
            
        }
    }
    
    void CRoadWidth::Rotate(const Mat &srcImage, Mat &destImage, double angle)
    {
        angle=angle*57.3;

        Point2f center(srcImage.cols / 2, srcImage.rows / 2);//中心

        Mat M = getRotationMatrix2D(center, angle, 1);//计算旋转的仿射变换矩阵

        warpAffine(srcImage, destImage, M, Size(srcImage.cols, srcImage.rows), INTER_LINEAR, BORDER_CONSTANT, Scalar(255));//仿射变换

        //circle(destImage, center, 2, Scalar(255, 255, 0));

    }

    void CRoadWidth::findReferencePixel(const double& angle,vector<CvPoint>& pixels){
        if(state_==1&&paramRW_.mode==1){
            CvPoint pixel;
            point2D2pixel( Point2d( pose_.x , pose_.y) , angle , pixel );
            pixels.push_back(pixel);
        }
        else if(state_==1&&paramRW_.mode==2){
            if(paramRW_.debugBool==1) cout<<"[findReferencePixel]******11111******"<<endl;
            for(auto slot:pldSlots_.slots){
                Point2d referencePoint , tmp1( (slot.p0.x+slot.p1.x)/2, (slot.p0.y+slot.p1.y)/2 ), tmp2( (slot.p2.x+slot.p3.x)/2, (slot.p2.y+slot.p3.y)/2 );
                CvPoint referencePixel;
                referencePoint.x = tmp1.x+(tmp1.x-tmp2.x)/3;
                referencePoint.y = tmp1.y+(tmp1.y-tmp2.y)/3;
                point2D2pixel(referencePoint,angle,referencePixel);
                pixels.push_back(referencePixel);
               // publishKeyPoint(referencePoint);
            }
        }
        else if(state_==2){
            Point2d referencePoint,tmp1((slotSelected_.p0.x+slotSelected_.p1.x)/2,(slotSelected_.p0.y+slotSelected_.p1.y)/2);
            Point2d tmp2((slotSelected_.p2.x+slotSelected_.p3.x)/2,(slotSelected_.p2.y+slotSelected_.p3.y)/2);
            CvPoint referencePixel;
            referencePoint.x=tmp1.x+(tmp1.x-tmp2.x)/3;
            referencePoint.y=tmp1.y+(tmp1.y-tmp2.y)/3;
            point2D2pixel(referencePoint,angle,referencePixel);
            pixels.push_back(referencePixel);
        }
        else{
            cout<<"[findReferencePixel] :state should be 1 or 2"<<endl;
            exit(1);
        }

    }

    void CRoadWidth::point2D2pixel(const Point2d& point2D, const double& angle, CvPoint &pixel){
        
        Vector3d pointMap,pointWorld;
        Matrix3d Tw2m,Tm2w;
        
        Tw2m<<cos(mapCoord_[2]) , -sin(mapCoord_[2]) , mapCoord_[0] , sin(mapCoord_[2]) , cos(mapCoord_[2]) , mapCoord_[1] , 0 , 0 , 1;
        Tm2w=Tw2m.inverse();
        
        pointWorld<<point2D.x,point2D.y,1;
        pointMap=Tm2w*pointWorld;    //将点转化为在gridmap坐标系的位置

        int x,y;
        x=(int)(pointMap.x()/paramRW_.scale);    //计算点在map_中的像素位置
        y=map_.cols-(int)(pointMap.y()/paramRW_.scale);

        CvPoint center,pixelRaw;
        center.x=map_.cols / 2;
        center.y=map_.rows/2;
        pixelRaw.x=x, pixelRaw.y=y;


        pixel=getPointAffinedPos(pixelRaw,center,angle);  //经过旋转后，点在旋转后的map的像素位置
#ifdef offline_ros
        publishKeyPoint(point2D);
#endif
//        while(1){
//            if( pixel.x<0 || pixel.y<0 || pixel.x>map_.cols || pixel.y>map_.rows){
//                pixel.x = pixel.x+(map_.cols/2-pixel.x)/10;
//                pixel.y = pixel.y+(map_.rows/2-pixel.y)/10;
//            }
//            else break;
//        }
//
//        cout<<"pixel.x: "<<pixel.x<<" ; pixel.y: "<<pixel.y<<endl;
//        cout<<"point2D.x: "<<point2D.x<<" ;point2D.y: "<<point2D.y<<endl;
//        assert(pixel.x>=0&&pixel.x<=map_.cols);
//        assert(pixel.y>=0&&pixel.y<=map_.rows);

    }

    void CRoadWidth::pixel2world(const CvPoint& pixel, array<double, 2>& worldPose){
        worldPose[0]=mapCoord_[0]+pixel.x*10;
        worldPose[1]=mapCoord_[1]+map_.rows*10-pixel.y*10;
    }

    void CRoadWidth::world2veh(const array<double,2>& worldPose, array<double,2>& mapPose){
        mapPose[0]=worldPose[0]-pose_.x;
        mapPose[1]=worldPose[1]-pose_.y;
    }

    CvPoint CRoadWidth::getPointAffinedPos(const CvPoint &src, const CvPoint &center, double angle){

        CvPoint dst;

        int x = src.x - center.x;

        int y = src.y - center.y;

        dst.x = cvRound(x * cos(angle) + y * sin(angle) + center.x);

        dst.y = cvRound(-x * sin(angle) + y * cos(angle) + center.y);

        return dst;
    }

    double CRoadWidth::roadWidthNoTraverse(Mat& mat,CvPoint& pixel,int left,int right,const int number,const double angle, int& top, int& bottom){
        if(paramRW_.debugBool==1) cout<<"[roadWidthNoTraverse]******1111*****"<<endl;
        top=min ( max ( pixel.y - int((paramVeh_.width-100)/paramRW_.scale/2) , 1 ) , mat.rows-10 );
        bottom=max( min( pixel.y + int((paramVeh_.width-100)/paramRW_.scale/2) , mat.rows-1 ) , 10 );
        if(paramRW_.debugBool==1)  cout<<"pixel.x: "<<pixel.x<<" pixel.y: "<<pixel.y<<" top: "<<top<<" bottom:"<<bottom<<" left:" <<left<<" right"<<right<<endl;
        Mat cut=mat(Range(top,bottom),Range(left,right));
        Point2d p0, p1, p2, p3;
        CvPoint pixelP0 , pixelP1,  pixelP2, pixelP3;
        if(paramRW_.debugBool==1) cout<<"[roadWidthNoTraverse]******1111222222*****"<<endl;
        if(state_==1&&paramRW_.mode==2){
            p0 = Point2d(pldSlots_.slots[number].p0.x , pldSlots_.slots[number].p0.y);
            p1 = Point2d(pldSlots_.slots[number].p1.x , pldSlots_.slots[number].p1.y);
            point2D2pixel(p0 , angle , pixelP0);
            point2D2pixel(p1 , angle , pixelP1);
        }
        if(paramRW_.debugBool==1) cout<<"[roadWidthNoTraverse]******2222*****"<<endl;
        if(!isFreeRegion(cut)){
            cout<<"[roadWidthNoTraverse] : 计算通车道失败"<<endl;
            return 0;
        }
        else{
            while(true){
                if(paramRW_.debugBool==1) cout<<"[roadWidthNoTraverse]******3333*****"<<endl;
                cut=mat(Range(top,bottom) , Range(left,right));
                if(state_==1&&paramRW_.mode==1){
                    if(!isFreeRegion(cut)||top<=1){
                        top+=1;
                        if(paramRW_.debugBool==1)  cout<<"topStart :"<<top<<endl;
                        break;
                    }
                    top-=1;
                }
                else if(state_==1&&paramRW_.mode==2){
                    if( validInclude(top,bottom,left,right,pixelP0,pixelP2) || validInclude(top,bottom,left,right,pixelP1,pixelP3) ||  !isFreeRegion(cut) || top<=1){
                        top+=1;
                        if(paramRW_.debugBool==1) cout<<" [mode2] topStart :"<<top<<endl;
                        break;
                    }
                    top-=1;
                }
                if(bottom - top > ( paramRW_.maxRoadWidth/paramRW_.scale )) break;
            }
            while(true){
                if(paramRW_.debugBool==1) cout<<"[roadWidthNoTraverse]******4444*****"<<endl;
                cut=mat(Range(top,bottom),Range(left,right));
                if(state_==1&&paramRW_.mode==1){
                    if(!isFreeRegion(cut)||bottom>=mat.rows-1){
                        bottom-=1;
                        if(paramRW_.debugBool==1)cout<<"bottom :"<<bottom<<endl;
                        break;
                    }
                    bottom+=1;
                }
                else if(state_==1&&paramRW_.mode==2){
                    if( validInclude(top,bottom,left,right,pixelP0,pixelP2) || validInclude(top,bottom,left,right,pixelP1,pixelP3) || !isFreeRegion(cut) || bottom>=mat.rows-1){
                        bottom-=1;
                        if(paramRW_.debugBool==1)cout<<" [mode2] bottomStart :"<<bottom<<endl;
                        break;
                    }
                    bottom+=1;
                }
                if( bottom - top > ( paramRW_.maxRoadWidth/paramRW_.scale ) ) break;
            }
        }
        return (bottom-top)*paramRW_.scale;
    }

    bool CRoadWidth::isFreeRegion(Mat& mat){
        int i=0;
        for(int i=0;i<mat.rows;i++){
            for(int j=0;j<mat.cols;j++)
            {
                int count=mat.at<unsigned char>(i,j);
                if(count<=180){
                    if( i>1 )           count+=mat.at<unsigned char>(i-1,j);
                    if( i<mat.rows-1 )  count+=mat.at<unsigned char>(i+1,j);
                    if( j>1 )           count+=mat.at<unsigned char>(i,j-1);
                    if( j<mat.cols-1 )  count+=mat.at<unsigned char>(i,j+1);
                    if( count <=2*255 ) return false;
                }
                //if(i>=3) return false;
            }
        }
        return true;
    }

    bool CRoadWidth::validInclude(const int& top, const int& bottom, int left ,int right, const CvPoint& p1, const CvPoint& p2){
//        if (top<=p.y && p.y<=bottom && left<=p.x && p.x<=right)
//            return true;
        if( (top<p1.y && p1.y<bottom) || (top<p2.y && p2.y<bottom)) return true;
        return false;
    }

    void CRoadWidth::lane(){
        Mat cvtImg, binImg;
        if(map_.empty()) return;
        if(map_.channels()==3){
            cvtColor(map_, cvtImg, CV_BGR2GRAY);
        }
        else
            cvtImg=map_.clone();

        threshold(cvtImg, binImg, 100, 255, THRESH_BINARY);
        for(int i=0;i<binImg.rows;i++){
            for(int j=0; j<binImg.cols; j++){
                if(binImg.at<uchar>(i,j)<150)
                    binImg.at<uchar>(i,j)=255;
                else
                    binImg.at<uchar>(i,j)=0;
            }
        }
        vector<Vec4i> lines;
        HoughLinesP(binImg, lines, houghPara.rho, houghPara.theta, houghPara.threshold, houghPara.minLineLength, houghPara.maxLineGap);
        cout<<lines.size()<<endl;
        vector< array< array<double,2>,2 > > positions;
        for(auto l:lines){
            cout<<"start: "<<l[0]<<" "<<l[1]<<"; end: "<<l[2]<<" "<<l[3]<<endl;
            //line(map_,cv::Point(l[0],l[1]), cv::Point(l[2], l[3]), Scalar(0,0,0), 2, 8);
            CvPoint p1,p2;
            p1.x=l[0]; p1.y=l[1];
            p2.x=l[2]; p2.y=l[3];
            array<array<double, 2>,2> position;
            pixel2world(p1,position[0]);
            pixel2world(p2,position[1]);
            positions.push_back(position);

        }
#ifdef offline_ros
        publishLane(positions);
#endif
//        imwrite("/home/renjie/map.png",map_);
//        if(lines.size()>0)
//            exit(0);


    }

#ifdef soc
     bool CRoadWidth::getSlots(ApaSlotsInfo_T& slots){
        if(slotsContainer_.isNew()==true){
            slots=slotsContainer_.getData();
            return true;
        }else return false;
    }

//    bool CRoadWidth::getGlobalPose(ApaGlobalVehiclePose_T& dr){
//        if(globalPoseContainer_.isNew()==true){
//            dr=globalPoseContainer_.getData();
//            return true;
//        }else return false;
//    }

#endif
#ifdef offline_ros
    void CRoadWidth::publishLane(const vector< array< array<double , 2> , 2> >& points){
        visualization_msgs::MarkerArray markerArray;
        visualization_msgs::Marker line_strip;
        line_strip.header.frame_id = "/map";
        line_strip.header.stamp = ros::Time::now();
        line_strip.ns = "roadWidth";
        line_strip.action = visualization_msgs::Marker::DELETEALL;
        line_strip.pose.orientation.w = 1.0;
        int id = 1;
        markerArray.markers.push_back(line_strip);
        line_strip.action = visualization_msgs::Marker::ADD;

        line_strip.type = visualization_msgs::Marker::LINE_STRIP;

        // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
        line_strip.scale.x = 0.1;

        // Line strip is blue
        line_strip.color.b = 1.0;
        line_strip.color.a = 1.0;
        for(auto point2:points){
            line_strip.id=id;
            id++;
            line_strip.points.clear();
            for(auto point:point2){
                geometry_msgs::Point p;
                p.x=point[0]/100; p.y=point[1]/100;
                line_strip.points.push_back(p);
            }

            markerArray.markers.push_back(line_strip);
        }


        pubLine_.publish(markerArray);
    }

    void CRoadWidth::publishRW(const vector<int>& left, const vector<int>& right, const vector<int>& top, const vector<int>& bottom, const double& angle){

        CvPoint center;
        vector<CvPoint> pixelRaw1, pixelRaw2, pixelRaw3, pixelRaw4 , pixel1, pixel2, pixel3, pixel4;
        vector<array<double, 2>> p1, p2, p3, p4;
        center.x=map_.cols / 2;
        center.y=map_.rows/2;
        for(int i=0;i<left.size();i++){
            pixelRaw1.push_back( CvPoint( left[i], top[i] ));
            pixelRaw2.push_back( CvPoint( right[i], top[i] ));
            pixelRaw3.push_back( CvPoint( right[i], bottom[i] ));
            pixelRaw4.push_back( CvPoint( left[i], bottom[i] ));

            pixel1.push_back(getPointAffinedPos(pixelRaw1[i], center, -angle));
            pixel2.push_back(getPointAffinedPos(pixelRaw2[i], center, -angle));
            pixel3.push_back(getPointAffinedPos(pixelRaw3[i], center, -angle));
            pixel4.push_back(getPointAffinedPos(pixelRaw4[i], center, -angle));

            array<double, 2> p;
            pixel2world(pixel1[i], p); p1.push_back(p);
            pixel2world(pixel2[i], p); p2.push_back(p);
            pixel2world(pixel3[i], p); p3.push_back(p);
            pixel2world(pixel4[i], p); p4.push_back(p);
        }


        visualization_msgs::MarkerArray markerArray;
        visualization_msgs::Marker line_strip;
        line_strip.header.frame_id = "/map";
        line_strip.header.stamp = ros::Time::now();
        line_strip.ns = "roadWidth";
        line_strip.pose.orientation.w = 1.0;
        int id=0;
        line_strip.action = visualization_msgs::Marker::DELETEALL;
        markerArray.markers.push_back(line_strip);
        line_strip.action = visualization_msgs::Marker::ADD;

        line_strip.action = visualization_msgs::Marker::ADD;
        line_strip.type = visualization_msgs::Marker::LINE_STRIP;

        line_strip.color.b = 1.0;
        line_strip.color.a = 1.0;
        line_strip.scale.x = 0.1;

        for(int i=0;i<p1.size();i++){
            id++;
            geometry_msgs::Point p;
            line_strip.points.clear();
            p.x=p1[i][0]/100; p.y=p1[i][1]/100; line_strip.points.push_back(p);
            p.x=p2[i][0]/100; p.y=p2[i][1]/100; line_strip.points.push_back(p);
            p.x=p3[i][0]/100; p.y=p3[i][1]/100; line_strip.points.push_back(p);
            p.x=p4[i][0]/100; p.y=p4[i][1]/100; line_strip.points.push_back(p);
            line_strip.id=id;
            stringstream ss;
            string s;
            ss<<id;
            ss>>s;
            line_strip.text=s;
            markerArray.markers.push_back(line_strip);
        }

        //cout<<"p1.x: "<<p1[0]<<" p1.y:"<<p1[1]<<" ; p2.x: "<<p2[0]<<" p2.y:"<<p2[1]<<" ; p3.x: "<<p3[0]<<" p3.y:"<<p3[1]<<" ; p4.x: "<<p4[0]<<" p4.y:"<<p4[1]<<endl;

        pubRW_.publish(markerArray);

    }

    void CRoadWidth::publishKeyPoint(Point2d point){
        visualization_msgs::Marker line_strip;
        line_strip.header.frame_id = "/map";
        line_strip.header.stamp = ros::Time::now();
        line_strip.ns = "roadWidth";
        line_strip.pose.orientation.w = 1.0;
        static int id=0;
        line_strip.id=id;
        id++;
        line_strip.action = visualization_msgs::Marker::ADD;
        line_strip.type = visualization_msgs::Marker::POINTS;

        line_strip.color.b = 1.0;
        line_strip.color.a = 1.0;
        line_strip.scale.x = 0.2;
        line_strip.scale.y = 0.2;
        geometry_msgs::Point p;
        p.x=point.x/100;
        p.y=point.y/100;
        line_strip.points.push_back(p);

        pubPoint_.publish(line_strip);


    }
#endif
//    double CRoadWidth::dtc(double R,int side, int gear){    //左正1  右负-1    gear=-1,后退  ，gear=1,前进
//        vector<CvPoint> pixelVeh;
//        Point2d circle;
//        double alp;
//        if(side>0){
//            alp=pose_.z-PI/2;
//        }
//        else{
//            alp=pose_.z+PI/2;
//        }
//        circle.x=pose_.x-R*cos(alp);
//        circle.y=pose_.y-R*sin(alp);
//        cout<<"pose.x :"<<pose_.x <<"; pose.y :"<<pose_.y<<"pose.z :"<<pose_.z<<"; alp :"<<alp<<endl;
//        cout<<"circle :"<<circle.x<<"  "<<circle.y<<endl;
//        double step;
//        if(side*gear>0){
//            step=10/R;
//        }
//        else if(side*gear<0){
//            step=-10/R;
//        }
//        else{
//            cout<<"[dtc] : side*gear should isn't 0"<<endl;
//            exit(0);
//        }
//
//        for(int i=0;i<100;i++){
//            vector<Point2d> pVeh;
//            vector<CvPoint> pixelVeh;
//            double theta=alp+i*step;
//            Point3d pose;
//            pose.x=circle.x+R*cos(theta);
//            pose.y=circle.y+R*sin(theta);
//            if(side>0){
//                pose.z=theta+PI/2;
//            } else{
//                pose.z=theta-PI/2;
//            }
//            positionW(pose,pVeh);
//            for(auto p:pVeh){
//                CvPoint pixel;
//                point2D2pixel(p,0,pixel);
//                pixelVeh.push_back(pixel);
//            }
//            for(auto pixel:pixelVeh){
//                if(pixel.x>=_map.rows||pixel.y>=_map.cols){
//                    cout<<"[dtc] :  error pixel.x>=_map.rows||pixel.y>=_map.cols"<<endl;
//                    exit(0);
//                }
//                if(_map.at<uchar>(pixel.x,pixel.y)<180){
//                    cout<<"[dtc] : the CvPoint ("<<pixel.x<<" "<<pixel.y<<") < 180"<<" ; value is "<<(int) _map.at<uchar>(pixel.x,pixel.y)<<endl;
//                    return 10*(i-1);
//                }
//            }
//        }
//        return 1000;
//    }
    
}
