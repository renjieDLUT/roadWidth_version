//
// Created by renjie on 2021/2/23.
//

#ifndef STD_DEMO_GRIDMAPEXCEPTION_H
#define STD_DEMO_GRIDMAPEXCEPTION_H
#include<stdexcept>
#include<iostream>
namespace Exception{

    class GridMapException :public std::logic_error {
    public:
        GridMapException(int width, int height):logic_error("invalid gridMap"){
            width_=width;
            height_=height;
        }
        void print(){
            std::cout<<"[gridMap] width: "<<width_<<" ;  height:"<<height_<<std::endl;
        }

    private:
        int width_;
        int height_;
    };

}



#endif //STD_DEMO_GRIDMAPEXCEPTION_H
