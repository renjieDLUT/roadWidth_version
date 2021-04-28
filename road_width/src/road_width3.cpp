
#include"road_width3.h"

namespace ca{

    CRoadWidth::CRoadWidth(int state,  Mat map, Point3d pose, vector<Slot> slots, array<double ,3> parkCoord, array<double ,3> mapCoord,
                           double scale, double mFront, double mRear, double thresholdAngle, double maxRoadWidth, int mode,Vehicle veh){
        _state=state;
        _map=map;
        _pose=pose;
        _slots=slots;
        _parkCoord=parkCoord;
        _mapCoord=mapCoord;


        _scale=scale;
        _mFront=mFront;
        _mRear=mRear;
        _thresholdAngle=thresholdAngle;
        _maxRoadWidth=maxRoadWidth;
        _mode=mode;
        _veh=veh;
    }

    CRoadWidth::CRoadWidth(int state, double scale, double mFront, double mRear, double thresholdAngle, double maxRoadWidth, int mode,Vehicle veh){
        _state=state;
        _scale=scale;
        _mFront=mFront;
        _mRear=mRear;
        _thresholdAngle=thresholdAngle;
        _maxRoadWidth=maxRoadWidth;
        _mode=mode;
        _veh=veh;
    }

    CRoadWidth::CRoadWidth(int state, array<double ,3> parkCoord, array<double ,3> mapCoord, Slot pointSelectSlot, int flagSlotSide,
                           double scale, double mFront, double mRear, double thresholdAngle, double maxRoadWidth, int mode,Vehicle veh){
        _state=state;
        _parkCoord=parkCoord;
        _mapCoord=mapCoord;
        _pointSelectSlot=pointSelectSlot;
        _flagSlotSide=flagSlotSide;

        _scale=scale;
        _mFront=mFront;
        _mRear=mRear;
        _thresholdAngle=thresholdAngle;
        _maxRoadWidth=maxRoadWidth;
        _mode=mode;
        _veh=veh;
    }

    void CRoadWidth::start() {
        _gridmap_subscriber=zros::core::Transport::Instance()->CreateSubscriber("fused_grid_map.topic");
        _globalPose_subscriber=zros::core::Transport::Instance()->CreateSubscriber("ca_apa_car_pose.topic");

        _gridmap_subscriber->RegisterCallback<ExtendedOccupancyGrid>(
                [this](const std::string& topic,const  ExtendedOccupancyGrid& msg )
                {

                    //cout<<"callback function gridmap"<<endl;
                    m1_.lock();
                    this->_flag1=true;
                    this->_gridmap=msg;
                    //this->toMat();
                    m1_.unlock();
                }
        );
        _globalPose_subscriber->RegisterCallback<ApaGlobalVehiclePose_T>(
                [this](const std::string& topic,const ApaGlobalVehiclePose_T& msg){
                    m2_.lock();
                    this->_globalPose=msg;
                    this->_flag2=true;
//                    this->_pose.x=(double)msg.globalvehx();
//                    this->_pose.y=(double)msg.globalvehy();
//                    this->_pose.z=(double)msg.globalvehangle();
                    m2_.unlock();
                }
        );
        _gridmap_subscriber->Start();
        _globalPose_subscriber->Start();
    }

    void CRoadWidth::setState(int state){
        _state=state;
    }

//void CRoadWidth::setGridmap(ExtendedOccupancyGrid gridmap){
//    toMat();
//}

    void CRoadWidth::setMap(Mat map){
        _map=map;
    }

    void CRoadWidth::setPose(Point3d pose){
        _pose=pose;
    }

    void CRoadWidth::setSlots(vector<Slot> slots){
        _slots=slots;
    }

    void CRoadWidth::setParkCoord(array<double ,3> parkCoord){
        _parkCoord=parkCoord;
    }

    void CRoadWidth::setMapCoord(array<double, 3> mapCoord) {
        _mapCoord=mapCoord;
    }

    void CRoadWidth::setIDTrackSlot(int idTrackSlot){
        _idTrackSlot=idTrackSlot;
    }

    void CRoadWidth::setSelectSlot(Slot pointSelectSlot){
        _pointSelectSlot=pointSelectSlot;
    }

    vector<double> CRoadWidth::getRoadWidth() {

	    bool flag1=false,flag2=false;
	    m1_.lock();
        if(_flag1== true){
            toMat();
            flag1=true;
        }
        m1_.unlock();
        m2_.lock();
        if(_flag2==true){
            _pose.x=(double)_globalPose.globalvehx();
            _pose.y=(double)_globalPose.globalvehy();
            _pose.z=(double)_globalPose.globalvehangle()/180*PI;
            flag2=true;
        }
        m2_.unlock();
	    if(flag2&&flag1){
	        cout<<"[getRoadWidth] : enter the function roadWidth"<<endl;
	        roadWidth();
	    }

        return _roadWidth;
    }

    void CRoadWidth::toMat(){
        std::vector<float> prob_free(_gridmap.info().width() * _gridmap.info().height());
        //cout<<"*************function tomat  111111111111111**********"<<endl;
        _map.create(_gridmap.info().height(),_gridmap.info().width(),CV_8UC1);
        share_buffer_free = zros::core::SharedBuffer::ImportBuffer(_gridmap.free_prob_tag());
        //cout<<"*************function tomat  222222222222222**********"<<endl;
        void* ptr_free = reinterpret_cast<void *>(share_buffer_free->GetPointer());
        memcpy(prob_free.data(), ptr_free, prob_free.size() * sizeof(float));
        //cout<<"prob_free.size() :"<<prob_free.size()<<endl;
        for(int i=0;i<_gridmap.info().height();i++){
            for(int j=0;j<_gridmap.info().width();j++){
                int tmp=i*_gridmap.info().width()+j;
                if(prob_free[tmp]>0.6){
                    _map.at<uchar>(_gridmap.info().height()-1-i,j)=255;
                }
                else{
                    _map.at<uchar>(_gridmap.info().height()-1-i,j)=0;
                }

            }
        }
        _mapCoord[0]=_gridmap.info().origin().position().x()*100;
        _mapCoord[1]=_gridmap.info().origin().position().y()*100;
        _mapCoord[2]=0;
    }

    void CRoadWidth::point2D2pixel(Point2d point2D,double angle,CvPoint &pixel){
        Vector3d pointMap,pointWorld;
        Matrix3d Tw2m,Tm2w;
        Tw2m<<cos(_mapCoord[2]),-sin(_mapCoord[2]),_mapCoord[0],sin(_mapCoord[2]),cos(_mapCoord[2]),_mapCoord[1],0,0,1;

        //cout<<"Tw2m :"<<endl<<Tw2m<<endl;

        Tm2w=Tw2m.inverse();
        pointWorld<<point2D.x,point2D.y,1;
        pointMap=Tm2w*pointWorld;    //将点转化为在gridmap坐标系的位置

        int u1,v1;
        u1=_map.rows-(int)(pointMap.x()/_scale);    //计算点在_map中的像素位置
        v1=(int)(pointMap.y()/_scale);

        CvPoint center;
        center.x=_map.cols / 2;
        center.y=_map.rows/2;
        pixel.x=u1, pixel.y=v1;

        getPointAffinedPos(pixel,center,angle);  //经过_rotateAngle旋转后，点在旋转后的map的像素位置

    }

    vector<double> CRoadWidth::linspace(double start,double end,int num){
        vector<double> result;
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
        return result;
    }

    void CRoadWidth::roadWidth(){
        //toMat();
        double baseAngle;
        vector<double> angles;
        Point2d vehPoint;
        vehPoint.x = _pose.x;
        vehPoint.y = _pose.y;

        if(_state==1){
            baseAngle=_pose.z;
        }
        else if(_state==2){
            baseAngle=_parkCoord[2];
        }else{
            cout<<"[roadWidth] : no this case"<<endl;
            exit(0);
        }

        cout<<"[roadWidth] : enter linspace function"<<endl;
        angles=linspace(baseAngle-_thresholdAngle,baseAngle+_thresholdAngle,_NUM);
        cout<<"angles :";
        for(auto tmp:angles){
            cout<<tmp<<"  ";
        }
        cout<<endl;
        vector<vector<double>> setRoadWidth;

        for(auto angle:angles){
            Mat destImage;
            vector<double> roadWidth;
            vector<CvPoint> referencePixel;

            Rotate(_map,destImage,angle);
            cout<<"[roadWidth] :enter function findReferencePixel"<<endl;
            findReferencePixel(angle,referencePixel);
            if(((_state==1&&_mode==1)||_state==2)&&!(referencePixel.size()==1)){
                cout<<"[roadWidth] : state==1,mode==1 or state==2 ,referencePixel.size should be 1"<<endl;
                exit(1);
            }
            for(auto pixel:referencePixel){
                int left,right;
                left=pixel.x-(int)(_mFront/_scale);
                right=pixel.x+(int)(_mRear/_scale);
                cout<<"angle :"<<angle<<endl;
                double tmpRoadWidth=roadWidthNoTraverse(destImage,pixel,left,right);
                roadWidth.push_back(tmpRoadWidth);
                cout<<"tmpRoadWidth :"<<tmpRoadWidth<<endl;
            }
            setRoadWidth.push_back(roadWidth);
        }
        _roadWidth=setRoadWidth[0];
        for(int i=0;i<setRoadWidth.size();i++){
            for(int j=0;j<setRoadWidth[i].size();j++){
                if(_roadWidth[j]<setRoadWidth[i][j]){
                    _roadWidth[j]=setRoadWidth[i][j];
                }
            }

        }

    }

    void CRoadWidth::findReferencePixel(double angle,vector<CvPoint>& pixels){
        if(_state==1&&_mode==1){
            CvPoint pixel;
            cout<<"[findReferencePixel] :enter function point2D2Pixel"<<endl;
            cout<<"the vehicle position :"<<_pose.x<<" "<<_pose.y<<endl;
            point2D2pixel(Point2d(_pose.x,_pose.y),angle,pixel);
            cout<<"the vehicle pixel :"<<pixel.x<<" "<<pixel.y<<endl;
            pixels.push_back(pixel);
        }
        else if(_state==1&&_mode==2){
            for(auto slot:_slots){
                Point2d referencePoint,tmp1((slot[0].x+slot[1].x)/2,(slot[0].y+slot[1].y)/2),tmp2((slot[2].x+slot[3].x)/2,(slot[2].y+slot[3].y)/2);
                CvPoint referencePixel;
                referencePoint.x=tmp1.x+(tmp1.x-tmp2.x)/3;
                referencePoint.y=tmp1.y+(tmp1.y-tmp2.y)/3;
                point2D2pixel(referencePoint,angle,referencePixel);
                pixels.push_back(referencePixel);
            }
        }
        else if(_state==2){
            Point2d referencePoint,tmp1((_pointSelectSlot[0].x+_pointSelectSlot[1].x)/2,(_pointSelectSlot[0].y+_pointSelectSlot[1].y)/2);
            Point2d tmp2((_pointSelectSlot[2].x+_pointSelectSlot[3].x)/2,(_pointSelectSlot[2].y+_pointSelectSlot[3].y)/2);
            CvPoint referencePixel;
            referencePoint.x=tmp1.x+(tmp1.x-tmp2.x)/3;
            referencePoint.y=tmp1.y+(tmp1.y-tmp2.y)/3;
            point2D2pixel(referencePoint,angle,referencePixel);
            pixels.push_back(referencePixel);
        }
        else{
            cout<<"[findReferencePixel] :state should be 1 or 2"<<endl;
        }

    }

    void CRoadWidth::Rotate(const Mat &srcImage, Mat &destImage, double angle)
    {
        angle=angle*57.3;

        Point2f center(srcImage.cols / 2, srcImage.rows / 2);//中心

        Mat M = getRotationMatrix2D(center, angle, 1);//计算旋转的仿射变换矩阵

        warpAffine(srcImage, destImage, M, Size(srcImage.cols, srcImage.rows));//仿射变换

        //circle(destImage, center, 2, Scalar(255, 255, 0));

    }

    CvPoint CRoadWidth::getPointAffinedPos(const CvPoint &src, const CvPoint &center, double angle){

        CvPoint dst;

        int x = src.x - center.x;

        int y = src.y - center.y;

        dst.x = cvRound(x * cos(angle) + y * sin(angle) + center.x);

        dst.y = cvRound(-x * sin(angle) + y * cos(angle) + center.y);

        return dst;
    }

    bool CRoadWidth::isFreeRegion(Mat mat){
        for(int i=0;i<mat.rows;i++){
            for(int j=0;j<mat.cols;j++)
            {
                if(mat.at<unsigned char>(i,j)<=180)
                    return false;
            }
        }
        return true;
    }

    double CRoadWidth::roadWidthNoTraverse(Mat& mat,CvPoint& pixel,int left,int right){

        int top=max(pixel.y-(int)((_veh.width+60)/_scale/2),1);
        int bottom=min(pixel.y+(int)((_veh.width+60)/_scale/2),mat.rows);
        Mat cut=mat(Range(top,bottom),Range(left,right));
        if(!isFreeRegion(cut)){
            cout<<"[roadWidthNoTraverse] : 计算通车道失败"<<endl;
            return 0;
        }else{
            while(true){
                cut=mat(Range(top-1,bottom),Range(left,right));
                if(!isFreeRegion(cut)||top<=1){
                    cout<<"topStart :"<<top<<endl;
                    break;
                }
                top-=1;
            }
            while(true){
                cut=mat(Range(top,bottom+1),Range(left,right));
                if(!isFreeRegion(cut)||top>=mat.rows-1){
                    cout<<"bottom :"<<top<<endl;
                    break;
                }
                bottom+=1;
            }
        }
        return (bottom-top)*_scale;
    }

    double CRoadWidth::dtc(double R,int side, int gear){    //左正1  右负-1    gear=-1,后退  ，gear=1,前进
        if(_flag1==false||_flag2==false)return 0;
        //vector<Point2d> pVeh;
        vector<CvPoint> pixelVeh;
        Point2d circle;
        double alp;
        if(side>0){
            alp=_pose.z-PI/2;
        }
        else{
            alp=_pose.z+PI/2;
        }
        circle.x=_pose.x-R*cos(alp);
        circle.y=_pose.y-R*sin(alp);
        cout<<"pose.x :"<<_pose.x <<"; pose.y :"<<_pose.y<<"pose.z :"<<_pose.z<<"; alp :"<<alp<<endl;
        cout<<"circle :"<<circle.x<<"  "<<circle.y<<endl;
        double step;
        if(side*gear>0){
            step=10/R;
        }
        else if(side*gear<0){
            step=-10/R;
        }
        else{
            cout<<"[dtc] : side*gear should isn't 0"<<endl;
            exit(0);
        }

        for(int i=0;i<100;i++){
            vector<Point2d> pVeh;
            vector<CvPoint> pixelVeh;
            double theta=alp+i*step;
            Point3d pose;
            pose.x=circle.x+R*cos(theta);
            pose.y=circle.y+R*sin(theta);
            if(side>0){
                pose.z=theta+PI/2;
            } else{
                pose.z=theta-PI/2;
            }
            positionW(pose,pVeh);
            for(auto p:pVeh){
                CvPoint pixel;
                point2D2pixel(p,0,pixel);
                pixelVeh.push_back(pixel);
            }
            for(auto pixel:pixelVeh){
                if(pixel.x>=_map.rows||pixel.y>=_map.cols){
                    cout<<"[dtc] :  error pixel.x>=_map.rows||pixel.y>=_map.cols"<<endl;
                    exit(0);
                }
                if(_map.at<uchar>(pixel.x,pixel.y)<180){
                    cout<<"[dtc] : the CvPoint ("<<pixel.x<<" "<<pixel.y<<") < 180"<<" ; value is "<<(int) _map.at<uchar>(pixel.x,pixel.y)<<endl;
                    //cout<<"gridmap value is"<<
                    return 10*(i-1);
                }
            }
        }
        return 1000;

    }

    void CRoadWidth::positionW(Point3d pose,vector<Point2d>& pVeh){
        for(int i=-_veh.rear;i<=(_veh.length-_veh.front);i+=10){
            for(int j=-_veh.width/2;j<=_veh.width/2;j+=10){
                Vector3d pointWorld,pointVeh;
                pointVeh<<i,j,1;
                Matrix3d Tw2v;
                Tw2v<<cos(pose.z),-sin(pose.z),pose.x,sin(pose.z),cos(pose.z),pose.y,0,0,1;
                pointWorld=Tw2v*pointVeh;
                pVeh.push_back(Point2d(pointWorld.x(),pointWorld.y()));
            }
        }
    }
}
